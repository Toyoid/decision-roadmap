import os
import time
import random
import numpy as np
from copy import deepcopy
import argparse
import rospy

import torch
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter

from config_nav_uniform_graph_sac import get_config_uniform_graph
from envs.nav_uniform_graph_env import NavUniformGraphEnv
from algorithm.attention_networks import PolicyNet, SoftQNetwork
from algorithm.offpolicy_replay_buffer import RoadmapReplayBuffer
from utils import get_time_str

import tyro
from dataclasses import dataclass
from distutils.util import strtobool

@dataclass
class InputArgs:
    robot_ns: str = ""
    drm_prefix: str = "dep"
    map_name: str = "maze_medium"
    episode_start: int = 0
    load_as_init: lambda x: bool(strtobool(x)) = True
    checkpoint_file: str = 'context_aware_nav/checkpoint.pth'

def main():
    rospy.init_node("decision_roadmap_agent")
    parsed_args = tyro.cli(InputArgs)
    robot_namespace = parsed_args.robot_ns
    drm_name = parsed_args.drm_prefix
    map_name = parsed_args.map_name
    episode_start = parsed_args.episode_start
    load_as_init = parsed_args.load_as_init
    checkpoint_file = parsed_args.checkpoint_file

    args = get_config_uniform_graph()
    args.map_name = map_name
    args.exp_name = os.path.basename(__file__)[: -len(".py")]
    current_time = get_time_str()
    run_name = f"{args.env_name}__{args.exp_name}__seed-{args.seed}__{current_time}"
    # init logger
    if args.use_wandb:
        import wandb

        wandb.init(
            project=args.wandb_project_name,
            entity=args.wandb_entity,
            sync_tensorboard=True,
            config=vars(args),
            name=run_name,
            monitor_gym=False,
            save_code=True,
        )
    writer = SummaryWriter(f"runs/{run_name}")
    writer.add_text(
        "hyperparameters",
        "|param|value|\n|-|-|\n%s" % ("\n".join([f"|{key}|{value}|" for key, value in vars(args).items()])),
    )

    # seeding
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    # cuda
    if args.cuda and torch.cuda.is_available():
        args.device = torch.device("cuda:0")
        torch.backends.cudnn.deterministic = args.cuda_deterministic
    else:
        args.device = torch.device("cpu")
    print(40 * "-")
    print("  Use device: ", args.device)
    print(40 * "-")
    torch.set_num_threads(args.n_training_threads)

    # env setup
    envs = NavUniformGraphEnv(
        args=args, 
        robot_namespace=robot_namespace, 
        drm_name=drm_name, 
        is_train=True
    )

    # define actor-critic networks
    actor = PolicyNet(args.input_dim, args.embedding_dim).to(args.device)
    qf1 = SoftQNetwork(args.input_dim, args.embedding_dim).to(args.device)
    qf2 = SoftQNetwork(args.input_dim, args.embedding_dim).to(args.device)
    qf1_target = SoftQNetwork(args.input_dim, args.embedding_dim).to(args.device)
    qf2_target = SoftQNetwork(args.input_dim, args.embedding_dim).to(args.device)

    if load_as_init:
        # load the model provided by "context-aware-navigation" as initialization
        # (https://github.com/marmotlab/Context_Aware_Navigation)
        print(f"[Load Model]: Load model from {args.model_path}/{checkpoint_file}")
        if args.device == 'cpu':
            checkpoint = torch.load(f'{args.model_path}/{checkpoint_file}', map_location=torch.device('cpu'))
        else:
            checkpoint = torch.load(f'{args.model_path}/{checkpoint_file}', map_location=args.device)
        actor.load_state_dict(checkpoint['policy_model'])
        qf1.load_state_dict(checkpoint['q_net1_model'])
        qf2.load_state_dict(checkpoint['q_net2_model'])

    qf1_target.load_state_dict(qf1.state_dict())
    qf2_target.load_state_dict(qf2.state_dict())
    q_optimizer = optim.Adam(list(qf1.parameters()) + list(qf2.parameters()), lr=args.q_lr)
    actor_optimizer = optim.Adam(list(actor.parameters()), lr=args.policy_lr)

    # automatic entropy tuning
    if args.autotune:
        target_entropy = 0.01 * (-np.log(1 / args.k_neighbor_size))  # TBD after reading SAC
        # log_alpha = torch.zeros(1, requires_grad=True, device=device)  # TBD after reading SAC
        log_alpha = torch.tensor([-2.], requires_grad=True, device=args.device)  # TBD after reading SAC
        alpha = log_alpha.exp().item()
        alpha_optimizer = optim.Adam([log_alpha], lr=1e-4)  # TBD after reading SAC: 1e-4 (context) q_lr in cleanrl
    else:
        alpha = args.alpha

    # set learning rate decay step | not use in context?
    actor_lr_decay = optim.lr_scheduler.StepLR(actor_optimizer, step_size=args.lr_decay_step, gamma=0.99)
    q_net_lr_decay = optim.lr_scheduler.StepLR(q_optimizer, step_size=args.lr_decay_step, gamma=0.99)

    # replay buffer
    replay_buffer = RoadmapReplayBuffer(args.buffer_size, args.device)

    # nav results
    time_cost_list = []
    dist_cost_list = []
    avg_time_cost = None
    avg_dist_cost = None

    # run the experiment
    print(80 * "*")
    print(f"  Robot {robot_namespace} Episode {episode_start}: ")
    print(80 * "*")
    global_step = 0
    episode_ita = 0
    start_time = time.time()
    while episode_ita < args.train_num_episodes:
        roadmap_state = envs.reset(episode_ita)
        while not rospy.is_shutdown():
            global_step += 1
            # interact with the environment and collect data
            node_inputs, edge_inputs, current_index, node_padding_mask, edge_padding_mask, edge_mask = roadmap_state
            if global_step <= args.learning_starts:
                # randomly sample actions
                flat_mask = edge_padding_mask.view(-1)
                action_space = torch.nonzero(flat_mask != 1).squeeze()
                action_index = torch.randint(0, len(action_space), (1,)).to(args.device)
            else:
                # sample action using actor policy
                with torch.no_grad():
                    log_pi_list = actor(node_inputs, edge_inputs, current_index, node_padding_mask, edge_padding_mask, edge_mask)
                    action_index = torch.multinomial(log_pi_list.exp(), 1).long().squeeze(1)

            next_roadmap_state, reward, done, info = envs.step(action_index)

            replay_buffer.add(deepcopy(roadmap_state), action_index.view(1, 1, 1).clone(), reward.clone(), done.clone(),
                              deepcopy(next_roadmap_state))
            roadmap_state = next_roadmap_state

            # training
            if global_step > args.learning_starts:
                print("[Training Info]: Training Step ", global_step)
                # TBD after empirical tuning : training for n times each step like: for j in range(8)
                for _ in range(2):
                    data = replay_buffer.sample(args.batch_size)

                    node_inputs_batch = data["node_inputs"]
                    edge_inputs_batch = data["edge_inputs"]
                    current_inputs_batch = data["current_index"]
                    node_padding_mask_batch = data["node_padding_mask"]
                    edge_padding_mask_batch = data["edge_padding_mask"]
                    edge_mask_batch = data["edge_mask"]
                    action_batch = data["action"]
                    reward_batch = data["reward"]
                    done_batch = data["done"]
                    next_node_inputs_batch = data["next_node_inputs"]
                    next_edge_inputs_batch = data["next_edge_inputs"]
                    next_current_inputs_batch = data["next_current_index"]
                    next_node_padding_mask_batch = data["next_node_padding_mask"]
                    next_edge_padding_mask_batch = data["next_edge_padding_mask"]
                    next_edge_mask_batch = data["next_edge_mask"]

                    with torch.no_grad():
                        next_log_pi = actor(next_node_inputs_batch, next_edge_inputs_batch, next_current_inputs_batch,
                                            next_node_padding_mask_batch, next_edge_padding_mask_batch, next_edge_mask_batch)
                        next_q1_values, _ = qf1_target(next_node_inputs_batch, next_edge_inputs_batch,
                                                       next_current_inputs_batch, next_node_padding_mask_batch,
                                                       next_edge_padding_mask_batch, next_edge_mask_batch)
                        next_q2_values, _ = qf2_target(next_node_inputs_batch, next_edge_inputs_batch,
                                                       next_current_inputs_batch, next_node_padding_mask_batch,
                                                       next_edge_padding_mask_batch, next_edge_mask_batch)
                        next_q_values = torch.min(next_q1_values, next_q2_values)
                        value_prime_batch = torch.sum(
                            next_log_pi.unsqueeze(2).exp() * (next_q_values - log_alpha.exp() * next_log_pi.unsqueeze(2)),
                            dim=1).unsqueeze(1)
                        target_q_batch = reward_batch + args.gamma * (1 - done_batch) * value_prime_batch
                        # ? equation doesn't align
                        # min_qf_next_target = torch.min(qf1_next_target, qf2_next_target) - alpha * next_state_log_pi
                        # next_q_value = data.rewards.flatten() + (1 - data.dones.flatten()) * args.gamma * (
                        #     min_qf_next_target).view(-1)

                    q1_values, _ = qf1(node_inputs_batch, edge_inputs_batch, current_inputs_batch,
                                       node_padding_mask_batch, edge_padding_mask_batch, edge_mask_batch)
                    q2_values, _ = qf2(node_inputs_batch, edge_inputs_batch, current_inputs_batch,
                                       node_padding_mask_batch, edge_padding_mask_batch, edge_mask_batch)
                    q1 = torch.gather(q1_values, 1, action_batch)
                    q2 = torch.gather(q2_values, 1, action_batch)
                    qf1_loss = F.mse_loss(q1, target_q_batch.detach())
                    qf2_loss = F.mse_loss(q2, target_q_batch.detach())
                    qf_loss = qf1_loss + qf2_loss

                    # optimize the model
                    q_optimizer.zero_grad()
                    qf_loss.backward()
                    # TBD:
                    # q1_grad_norm = torch.nn.utils.clip_grad_norm_(qf1.parameters(), max_norm=20000, norm_type=2)
                    # q2_grad_norm = torch.nn.utils.clip_grad_norm_(qf2.parameters(), max_norm=20000, norm_type=2)
                    q_optimizer.step()

                    if global_step % args.policy_frequency == 0:  # TD 3 Delayed update support
                        for _ in range(
                                args.policy_frequency
                        ):  # compensate for the delay by doing 'actor_update_interval' instead of 1
                            with torch.no_grad():
                                # using torch.no_grad() because the Q-Net doesn't take actor output as input
                                q1_values, _ = qf1(node_inputs_batch, edge_inputs_batch, current_inputs_batch,
                                                   node_padding_mask_batch, edge_padding_mask_batch, edge_mask_batch)
                                q2_values, _ = qf2(node_inputs_batch, edge_inputs_batch, current_inputs_batch,
                                                   node_padding_mask_batch, edge_padding_mask_batch, edge_mask_batch)
                                q_values = torch.min(q1_values, q2_values)

                            log_pi = actor(node_inputs_batch, edge_inputs_batch, current_inputs_batch,
                                           node_padding_mask_batch, edge_padding_mask_batch, edge_mask_batch)
                            actor_loss = torch.sum((log_pi.exp().unsqueeze(2) * (
                                        log_alpha.exp().detach() * log_pi.unsqueeze(2) - q_values.detach())), dim=1).mean()
                            # ? equation doesn't align
                            # actor_loss = ((alpha * log_pi) - min_qf_pi).mean()

                            actor_optimizer.zero_grad()
                            actor_loss.backward()
                            # TBD: actor_grad_norm = torch.nn.utils.clip_grad_norm_(actor.parameters(), max_norm=100, norm_type=2)
                            actor_optimizer.step()

                            if args.autotune:
                                entropy = (log_pi * log_pi.exp()).sum(dim=-1)
                                alpha_loss = -(log_alpha * (entropy.detach() + target_entropy)).mean()
                                # ? equation doesn't align
                                # with torch.no_grad():
                                #     _, log_pi, _ = actor.get_action(data.observations)
                                # alpha_loss = (-log_alpha.exp() * (log_pi + target_entropy)).mean()

                                alpha_optimizer.zero_grad()
                                alpha_loss.backward()
                                alpha_optimizer.step()
                                alpha = log_alpha.exp().item()

                # update learning rates
                q_net_lr_decay.step()
                if global_step % args.policy_frequency == 0:
                    actor_lr_decay.step()

                # update the target networks
                if global_step % args.target_network_frequency == 0:  # 64
                    for param, target_param in zip(qf1.parameters(), qf1_target.parameters()):
                        target_param.data.copy_(args.tau * param.data + (1 - args.tau) * target_param.data)  # soft or hard???
                    for param, target_param in zip(qf2.parameters(), qf2_target.parameters()):
                        target_param.data.copy_(args.tau * param.data + (1 - args.tau) * target_param.data)

                # save net parameters
                if args.save_model:
                    if (global_step % args.model_save_frequency == 0) or ((episode_ita == args.train_num_episodes - 1) and done):
                        save_dir = f"{args.model_path}/dynamic_ug_nav/{current_time}"
                        os.makedirs(save_dir, exist_ok=True)

                        checkpoint = {
                            "actor_network": actor.state_dict(),
                            "qf1_network": qf1.state_dict(),
                            "qf2_network": qf2.state_dict(),
                            "log_alpha": log_alpha,
                        }
                        path_checkpoint = f"{save_dir}/checkpoint_{episode_ita}.pth"
                        torch.save(checkpoint, path_checkpoint)
                        print(f'[Save Model]: Model weights on episode {episode_ita} saved in {path_checkpoint}\n')

                # record training data
                if global_step % 100 == 0:
                    writer.add_scalar("losses/qf1_values", q1.mean().item(), global_step)
                    writer.add_scalar("losses/qf2_values", q2.mean().item(), global_step)
                    writer.add_scalar("losses/qf1_loss", qf1_loss.item(), global_step)
                    writer.add_scalar("losses/qf2_loss", qf2_loss.item(), global_step)
                    writer.add_scalar("losses/qf_loss", qf_loss.item() / 2.0, global_step)
                    writer.add_scalar("losses/actor_loss", actor_loss.item(), global_step)
                    writer.add_scalar("losses/alpha", alpha, global_step)
                    writer.add_scalar("learning_rates/actor_lr", actor_optimizer.param_groups[0]['lr'], global_step)
                    writer.add_scalar("learning_rates/q-net_lr", q_optimizer.param_groups[0]['lr'], global_step)
                    # print("SPS:", int(global_step / (time.time() - start_time)))
                    # writer.add_scalar("charts/SPS", int(global_step / (time.time() - start_time)), global_step)
                    if args.autotune:
                        writer.add_scalar("losses/alpha_loss", alpha_loss.item(), global_step)

            # check the episode end points and log the relevant episodic return (not considering parallel envs)
            if info["episodic_outcome"] is not None:
                if info["episodic_outcome"] == "success" or info["episodic_outcome"] == "timeout":
                    time_cost_list.append(info['episodic_time_cost'])
                    dist_cost_list.append(info['episodic_dist_cost'])
                    avg_time_cost = sum(time_cost_list) / len(time_cost_list)
                    avg_dist_cost = sum(dist_cost_list) / len(dist_cost_list)

                print(f"[Training Info]: episode={episode_ita}, "
                      f"global_step={global_step}, outcome={info['episodic_outcome']}, "
                      f"episodic_return={info['episodic_return']:.2f}, \n"
                      f"episodic_length={info['episodic_length']}, \n"
                      f"episodic time cost={info['episodic_time_cost']:.3f} s, \n"
                      f"episodic distance cost={info['episodic_dist_cost']:.3f} m, \n"
                      f"success: {info['outcome_statistic']['success']}, "
                      f"collision: {info['outcome_statistic']['collision']}, "
                      f"timeout: {info['outcome_statistic']['timeout']}, "
                      f"success rate: {(100 * info['outcome_statistic']['success'] / (episode_ita - episode_start + 1)):.1f}% \n"
                      f"average time cost: {avg_time_cost:.3f} s, \n"
                      f"average distance cost: {avg_dist_cost:.3f} m, \n"
                      f"actor lr: {actor_optimizer.param_groups[0]['lr']}, "
                      f"q-net lr: {q_optimizer.param_groups[0]['lr']} \n")
                writer.add_scalar("charts/episodic_return", info["episodic_return"], global_step)
                writer.add_scalar("charts/episodic_length", info["episodic_length"], global_step)
                writer.add_scalar("charts/success_rate",
                                  info['outcome_statistic']['success'] / (episode_ita - episode_start + 1), global_step)
                writer.add_scalar("charts/episodic_time_cost", info['episodic_time_cost'], global_step)
                writer.add_scalar("charts/episodic_dist_cost", info['episodic_dist_cost'], global_step)
                episode_ita += 1
                if episode_ita < args.train_num_episodes:
                    print(80 * "*")
                    print(f"  Episode {episode_ita}: ")
                    print(80 * "*")
                break

    training_period = time.time() - start_time
    hours = int(training_period // 3600)
    minutes = int((training_period % 3600) // 60)
    seconds = int(training_period % 60)
    print(f"Total training time: {hours} h, {minutes} mins, {seconds} s")
    writer.close()
    envs.close()


if __name__ == "__main__":
    main()
