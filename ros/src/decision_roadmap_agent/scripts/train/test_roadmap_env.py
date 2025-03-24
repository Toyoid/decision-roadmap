# import os
import time
# from datetime import datetime
import random
import numpy as np
# import sys
# sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
import rospy

# import torch
# from torch.utils.tensorboard import SummaryWriter

# from config import get_config
from envs.nav_dynamic_prm_env import NavDynamicPRMEnv
''' change `e2e_navigation_env.py` to `e2e_navigation_env` '''
# from algorithm.ppo.ppo_uav_agent import PPOUAVAgent


# def get_time_str():
#     date_time = datetime.fromtimestamp(time.time())
#     formatted_date_time = date_time.strftime('%Y%m%d%H%M%S')
#     return formatted_date_time


def main():
    # args = get_config()
    # args.exp_name = os.path.basename(__file__)[: -len(".py")]
    # run_name = f"{args.env_name}__{args.exp_name}__seed-{args.seed}__{get_time_str()}"
    # # init logger
    # if args.use_wandb:
    #     import wandb
    #
    #     wandb.init(
    #         project=args.wandb_project_name,
    #         entity=args.wandb_entity,
    #         sync_tensorboard=True,
    #         config=vars(args),
    #         name=run_name,
    #         monitor_gym=False,
    #         save_code=True,
    #     )
    # writer = SummaryWriter(f"runs/{run_name}")
    # writer.add_text(
    #     "hyperparameters",
    #     "|param|value|\n|-|-|\n%s" % ("\n".join([f"|{key}|{value}|" for key, value in vars(args).items()])),
    # )
    #
    # # seeding
    # random.seed(args.seed)
    # np.random.seed(args.seed)
    # torch.manual_seed(args.seed)
    #
    # # cuda
    # if args.cuda and torch.cuda.is_available():
    #     print("choose to use gpu...")
    #     device = torch.device("cuda:0")
    #     torch.set_num_threads(args.n_training_threads)
    #     if args.cuda_deterministic:
    #         torch.backends.cudnn.benchmark = False
    #         torch.backends.cudnn.deterministic = True
    # else:
    #     print("choose to use cpu...")
    #     device = torch.device("cpu")
    #     torch.set_num_threads(args.n_training_threads)

    # env setup
    rospy.init_node("decision_roadmap_agent")
    envs = NavDynamicPRMEnv()
    # agent = PPOUAVAgent(envs, device, args)
    np.set_printoptions(precision=3)

    # run the experiment
    global_step = 0
    episode_ita = 0
    start_time = time.time()
    ''' record the total training time '''
    next_roadmap_state = envs.reset(episode_ita)  # need seeding?
    # next_robot_obs = torch.Tensor(next_robot_obs).to(device)  # (1, 5)
    # next_done = torch.zeros(args.num_envs).to(device)  # (1,)

    # while episode_ita < args.num_episodes:
    while episode_ita < 1000:
        # # annealing the rate if instructed to do so.
        # if args.anneal_lr:
        #     agent.anneal_lr(episode_ita + 1, args.num_episodes)

        # interact with the environment and collect data
        # for step in range(0, args.num_steps):
        for step in range(0, 128):
            # global_step += args.num_envs
            global_step += 1
            # agent.buffer.img_obs[step] = next_img_obs
            # agent.buffer.robot_obs[step] = next_robot_obs
            # agent.buffer.dones[step] = next_done
            # # ALGO LOGIC: action logic
            # with torch.no_grad():
            #     action, logprob, _, value, act_mean, act_std = agent.ac_net.get_action_and_value(next_img_obs, next_robot_obs)
            #     # for debugging
            #     if step % 20 == 0:
            #         print("{:<30} {}".format("[ACTION INFO] Action Mean: ", act_mean.cpu().numpy()))
            #         print("{:<30} {}".format("[ACTION INFO] Action Sampled: ", action.cpu().numpy()))
            #         print("{:<30} {}".format("[ACTION INFO] Action Std: ", act_std.cpu().numpy()))
            #         print("\n")
            #
            #     agent.buffer.values[step] = value.flatten()
            # agent.buffer.actions[step] = action  # action shape: (1, 3)
            # agent.buffer.logprobs[step] = logprob

            # Naive/Random Node Selection
            # action = max(next_roadmap_state["nodes"], key=lambda node: node.utility)
            action = random.choice(next_roadmap_state["nodes"])

            # execute the game and log data.
            next_roadmap_state, reward, next_done, info = envs.step(action)
            # agent.buffer.rewards[step] = torch.tensor(reward).to(device).view(-1)
            # next_img_obs, next_robot_obs = torch.Tensor(next_img_obs).to(device), torch.Tensor(next_robot_obs).to(device)
            # next_done = torch.Tensor(next_done).to(device)

            # check the episode end points and log the relevant episodic return (not considering parallel envs)
            if info["episodic_outcome"] is not None:
                print(f"[Training Info]: episode={episode_ita+1}, "
                      f"global_step={global_step}, outcome={info['episodic_outcome']}, "
                      f"episodic_return={info['episodic_return']}, episodic_length={info['episodic_length']}, "
                      f"success: {info['outcome_statistic']['success']}, "
                      f"collision: {info['outcome_statistic']['collision']}, "
                      f"timeout: {info['outcome_statistic']['timeout']}\n")
                # writer.add_scalar("charts/episodic_return", info["episodic_return"], global_step)
                # writer.add_scalar("charts/episodic_length", info["episodic_length"], global_step)
                # replace next_obs with the initial obs of the new episode,
                # but do not replace next_done with the new one, which will affect the advantage computation of the previous episode
                episode_ita += 1
                # if episode_ita == args.num_episodes:
                if episode_ita == 1000:
                    break
                next_roadmap_state = envs.reset(episode_ita)
                # next_img_obs, next_robot_obs = torch.Tensor(next_img_obs).to(device), torch.Tensor(next_robot_obs).to(device)

    #     # optimize policy and value networks
    #     agent.buffer.next_rollout_img = next_img_obs
    #     agent.buffer.next_rollout_rob_obs = next_robot_obs
    #     agent.buffer.next_rollout_done = next_done
    #     train_info = agent.update_nets()
    #
    #     # save net parameters
    #     '''save'''
    #
    #     # record rewards for plotting purposes
    #     v_loss, pg_loss, entropy_loss, old_approx_kl, approx_kl, clipfracs = train_info
    #     writer.add_scalar("charts/learning_rate", agent.optimizer.param_groups[0]["lr"], global_step)
    #     writer.add_scalar("losses/value_loss", v_loss.item(), global_step)
    #     writer.add_scalar("losses/policy_loss", pg_loss.item(), global_step)
    #     writer.add_scalar("losses/entropy", entropy_loss.item(), global_step)
    #     writer.add_scalar("losses/old_approx_kl", old_approx_kl.item(), global_step)
    #     writer.add_scalar("losses/approx_kl", approx_kl.item(), global_step)
    #     writer.add_scalar("losses/clipfrac", np.mean(clipfracs), global_step)
    #     # print("SPS:", int(global_step / (time.time() - start_time)))
    #     # writer.add_scalar("charts/SPS", int(global_step / (time.time() - start_time)), global_step)
    #
    # writer.close()


if __name__ == "__main__":
    main()
