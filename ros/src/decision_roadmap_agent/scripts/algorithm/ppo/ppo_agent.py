import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

from algorithm.ppo.ppo_networks import PPONetworks


class PPOAgent:
    """
    RL Agent learning with PPO
    """
    def __init__(self, envs, device, args):
        self.args = args
        self.device = device
        self.obs_shape = envs.single_observation_space.shape
        self.obs_dim = np.array(envs.single_observation_space.shape).prod()
        self.action_shape = envs.single_action_space.shape
        self.action_dim = np.prod(envs.single_action_space.shape)

        # actor-critic neural network
        self.ac_net = PPONetworks(self.obs_dim, self.action_dim).to(self.device)
        self.optimizer = optim.Adam(self.ac_net.parameters(), lr=self.args.learning_rate, eps=1e-5)

    def anneal_lr(self, iteration, num_iterations):
        frac = 1.0 - (iteration - 1.0) / num_iterations
        lrnow = frac * self.args.learning_rate
        self.optimizer.param_groups[0]["lr"] = lrnow

    def update_nets(self, data):
        """
        Optimize the policy and value networks using collected trajectories
        :return: training information
        """
        obs, actions, logprobs, rewards, dones, values, next_rollout_obs, next_rollout_done = data

        # bootstrap value if not done
        with torch.no_grad():
            next_value = self.ac_net.get_value(next_rollout_obs).reshape(1, -1)
            advantages = torch.zeros_like(rewards).to(self.device)
            lastgaelam = 0
            for t in reversed(range(self.args.num_steps)):
                if t == self.args.num_steps - 1:
                    nextnonterminal = 1.0 - next_rollout_done
                    nextvalues = next_value
                else:
                    nextnonterminal = 1.0 - dones[t + 1]
                    nextvalues = values[t + 1]
                delta = rewards[t] + self.args.gamma * nextvalues * nextnonterminal - values[t]
                advantages[t] = lastgaelam = delta + self.args.gamma * self.args.gae_lambda * nextnonterminal * lastgaelam
            returns = advantages + values

        # flatten the batch
        b_obs = obs.reshape((-1,) + self.obs_shape)
        b_logprobs = logprobs.reshape(-1)
        b_actions = actions.reshape((-1,) + self.action_shape)
        b_advantages = advantages.reshape(-1)
        b_returns = returns.reshape(-1)
        b_values = values.reshape(-1)

        # Optimizing the policy and value network
        clipfracs = []
        for epoch in range(self.args.update_epochs):
            b_inds = torch.randperm(self.args.batch_size, device=self.device)
            for start in range(0, self.args.batch_size, self.args.minibatch_size):
                end = start + self.args.minibatch_size
                mb_inds = b_inds[start:end]

                _, newlogprob, entropy, newvalue = self.ac_net.get_action_and_value(b_obs[mb_inds], b_actions[mb_inds])
                logratio = newlogprob - b_logprobs[mb_inds]
                ratio = logratio.exp()

                with torch.no_grad():
                    # calculate approx_kl http://joschu.net/blog/kl-approx.html
                    old_approx_kl = (-logratio).mean()
                    approx_kl = ((ratio - 1) - logratio).mean()
                    clipfracs += [((ratio - 1.0).abs() > self.args.clip_coef).float().mean().item()]

                mb_advantages = b_advantages[mb_inds]
                if self.args.norm_adv:
                    mb_advantages = (mb_advantages - mb_advantages.mean()) / (mb_advantages.std() + 1e-8)

                # Policy loss
                pg_loss1 = -mb_advantages * ratio
                pg_loss2 = -mb_advantages * torch.clamp(ratio, 1 - self.args.clip_coef, 1 + self.args.clip_coef)
                pg_loss = torch.max(pg_loss1, pg_loss2).mean()

                # Value loss
                newvalue = newvalue.view(-1)
                if self.args.clip_vloss:
                    v_loss_unclipped = (newvalue - b_returns[mb_inds]) ** 2
                    v_clipped = b_values[mb_inds] + torch.clamp(
                        newvalue - b_values[mb_inds],
                        -self.args.clip_coef,
                        self.args.clip_coef,
                    )
                    v_loss_clipped = (v_clipped - b_returns[mb_inds]) ** 2
                    v_loss_max = torch.max(v_loss_unclipped, v_loss_clipped)
                    v_loss = 0.5 * v_loss_max.mean()
                else:
                    v_loss = 0.5 * ((newvalue - b_returns[mb_inds]) ** 2).mean()

                entropy_loss = entropy.mean()
                loss = pg_loss - self.args.ent_coef * entropy_loss + v_loss * self.args.vf_coef

                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.ac_net.parameters(), self.args.max_grad_norm)
                self.optimizer.step()

            if self.args.target_kl is not None and approx_kl > self.args.target_kl:
                break

        # return training data for plotting
        train_info = v_loss, pg_loss, entropy_loss, old_approx_kl, approx_kl, clipfracs
        return train_info

