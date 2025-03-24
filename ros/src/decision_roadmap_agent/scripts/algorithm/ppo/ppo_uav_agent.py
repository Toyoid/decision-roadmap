import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from algorithm.ppo.ppo_networks import CNNPPONets


class RolloutBuffer:
    def __init__(self, num_steps, num_envs, img_obs_shape, robot_obs_shape, action_shape, device):
        self.img_obs = torch.zeros((num_steps, num_envs) + img_obs_shape).to(device)  # (num_steps, num_envs)+(10,480,640) = (num_steps, num_envs, 10, 480, 640)
        self.robot_obs = torch.zeros((num_steps, num_envs) + robot_obs_shape).to(device)  # (num_steps, num_envs)+(5,) = (num_steps, num_envs, 5)
        self.actions = torch.zeros((num_steps, num_envs) + action_shape).to(device)  # (num_steps, num_envs)+(3,) = (num_steps, num_envs, 3)
        self.logprobs = torch.zeros((num_steps, num_envs)).to(device)
        self.rewards = torch.zeros((num_steps, num_envs)).to(device)
        self.dones = torch.zeros((num_steps, num_envs)).to(device)
        self.values = torch.zeros((num_steps, num_envs)).to(device)
        self.next_rollout_img = None
        self.next_rollout_rob_obs = None
        self.next_rollout_done = None


class PPOUAVAgent:
    """
    UAV Control Agent learning with PPO
    """
    def __init__(self, envs, device, args):
        self.args = args
        self.device = device

        self.img_obs_shape = envs.img_obs_space
        self.robot_obs_shape = envs.robot_obs_space
        self.img_obs_dim = np.array(self.img_obs_shape).prod()
        self.robot_obs_dim = np.array(self.robot_obs_shape).prod()

        self.action_shape = envs.action_space
        self.action_dim = np.prod(self.action_shape)
        self.action_scale = torch.Tensor([[args.linear_spd_limit_x, args.linear_spd_limit_y, args.angular_spd_limit]]).to(self.device)

        self.buffer = RolloutBuffer(args.num_steps, args.num_envs, self.img_obs_shape, self.robot_obs_shape, self.action_shape, device)

        # actor-critic neural network
        self.ac_net = CNNPPONets(self.img_obs_shape, self.robot_obs_dim, self.action_dim, self.action_scale).to(self.device)
        self.optimizer = optim.Adam(self.ac_net.parameters(), lr=self.args.learning_rate, eps=1e-5)

    def anneal_lr(self, iteration, num_iterations):
        frac = 1.0 - (iteration - 1.0) / num_iterations
        lrnow = frac * self.args.learning_rate
        self.optimizer.param_groups[0]["lr"] = lrnow

    def update_nets(self):
        """
        Optimize the policy and value networks using collected trajectories
        :return: training information
        """
        # bootstrap value if not done
        with torch.no_grad():
            next_value = self.ac_net.get_value(self.buffer.next_rollout_img, self.buffer.next_rollout_rob_obs).reshape(1, -1)
            advantages = torch.zeros_like(self.buffer.rewards).to(self.device)
            lastgaelam = 0
            for t in reversed(range(self.args.num_steps)):
                if t == self.args.num_steps - 1:
                    nextnonterminal = 1.0 - self.buffer.next_rollout_done
                    nextvalues = next_value
                else:
                    nextnonterminal = 1.0 - self.buffer.dones[t + 1]
                    nextvalues = self.buffer.values[t + 1]
                delta = self.buffer.rewards[t] + self.args.gamma * nextvalues * nextnonterminal - self.buffer.values[t]
                advantages[t] = lastgaelam = delta + self.args.gamma * self.args.gae_lambda * nextnonterminal * lastgaelam
            returns = advantages + self.buffer.values

        # flatten the batch
        b_img_obs = self.buffer.img_obs.reshape((-1,) + self.img_obs_shape)
        b_robot_obs = self.buffer.robot_obs.reshape((-1,) + self.robot_obs_shape)
        b_logprobs = self.buffer.logprobs.reshape(-1)
        b_actions = self.buffer.actions.reshape((-1,) + self.action_shape)
        b_advantages = advantages.reshape(-1)
        b_returns = returns.reshape(-1)
        b_values = self.buffer.values.reshape(-1)

        # Optimizing the policy and value network
        clipfracs = []
        for epoch in range(self.args.update_epochs):
            b_inds = torch.randperm(self.args.batch_size, device=self.device)
            for start in range(0, self.args.batch_size, self.args.minibatch_size):
                end = start + self.args.minibatch_size
                mb_inds = b_inds[start:end]

                _, newlogprob, entropy, newvalue, _, _ = self.ac_net.get_action_and_value(b_img_obs[mb_inds], b_robot_obs[mb_inds], b_actions[mb_inds])
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

