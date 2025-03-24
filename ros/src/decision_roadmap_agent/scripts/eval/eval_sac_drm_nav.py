import os
import time
import random
import numpy as np
import rospy
import torch
import tyro
from dataclasses import dataclass

from config_nav_dynamic_prm_sac import get_config_dynamic_prm
from config_nav_fg_drm_sac import get_config_fg_drm
from envs.nav_dynamic_prm_env import NavDynamicPRMEnv
from envs.nav_fg_drm_env import NavFrontierGuidedDRMEnv
from algorithm.attention_networks import PolicyNet
from utils import record_nav_metrics, get_time_str

CONFIG_FUNC = {
    'dep': get_config_dynamic_prm,
    'fg_drm': get_config_fg_drm,
}

DRM_NAV_ENV = {
    'dep': NavDynamicPRMEnv,
    'fg_drm': NavFrontierGuidedDRMEnv,
}

@dataclass
class InputArgs:
    robot_ns: str = ""
    drm_prefix: str = "fg_drm"
    map_name: str = "maze_medium"
    episode_start: int = 0
    fg_drm_checkpoint_file: str = 'fg_drm_nav/20250314_011654_maze_medium/checkpoint_999.pth'
    dynamic_prm_checkpoint_file: str = 'dynamic_prm_nav/20250321_141526_maze_medium/checkpoint_999.pth'

def main():
    # tyro maintains a global registry of command-line parsers, so it should only be called once
    parsed_args = tyro.cli(InputArgs)  
    robot_namespace = parsed_args.robot_ns
    drm_name = parsed_args.drm_prefix
    map_name = parsed_args.map_name
    episode_start = parsed_args.episode_start
    fg_drm_checkpoint_file = parsed_args.fg_drm_checkpoint_file
    dynamic_prm_checkpoint_file = parsed_args.dynamic_prm_checkpoint_file

    args = CONFIG_FUNC[drm_name]()
    args.map_name = map_name
    args.exp_name = os.path.basename(__file__)[: -len(".py")]
    run_name = f"{args.env_name}__{args.exp_name}__seed-{args.seed}__{get_time_str()}"

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
    rospy.init_node("decision_roadmap_agent")
    envs = DRM_NAV_ENV[drm_name](
        args=args, 
        robot_namespace=robot_namespace, 
        drm_name=drm_name, 
        is_train=False
    )

    # define actor network
    actor = PolicyNet(args.input_dim, args.embedding_dim).to(args.device)

    # load actor checkpoint
    if drm_name == "dep":
        print(f"[Load Model]: Load Dynamic-PRM model from {args.model_path}/{dynamic_prm_checkpoint_file}")
        checkpoint = torch.load(f'{args.model_path}/{dynamic_prm_checkpoint_file}', map_location=args.device)
        metrics_csv_file = f"dynamic_prm_{map_name}_metrics_{get_time_str()}.csv"
    elif drm_name == "fg_drm":
        print(f"[Load Model]: Load FG-DRM model from {args.model_path}/{fg_drm_checkpoint_file}")
        checkpoint = torch.load(f'{args.model_path}/{fg_drm_checkpoint_file}', map_location=args.device)
        metrics_csv_file = f"fg_drm_{map_name}_metrics_{get_time_str()}.csv"
    else:
        raise ValueError("Unknown DRM name")
        
    actor.load_state_dict(checkpoint["actor_network"])

    # eval results
    time_cost_list = []
    dist_cost_list = []
    runtime_list = []
    success_rate = None
    avg_time_cost = None
    avg_dist_cost = None
    std_time_cost = None
    std_dist_cost = None

    # run the evaluation
    print(80 * "*")
    print(f"  Robot {robot_namespace} Episode {episode_start}: ")
    print(80 * "*")
    episode_ita = episode_start
    start_time = time.time()
    while episode_ita < args.eval_num_episodes:
        roadmap_state = envs.reset(episode_ita)
        while not rospy.is_shutdown():
            node_inputs, edge_inputs, current_index, node_padding_mask, edge_padding_mask, edge_mask = roadmap_state
            with torch.no_grad():
                log_pi_list = actor(node_inputs, edge_inputs, current_index, node_padding_mask, edge_padding_mask, edge_mask)
                
            if args.greedy:
                action_index = torch.argmax(log_pi_list, dim=1).long()
            else:
                action_index = torch.multinomial(log_pi_list.exp(), 1).long().squeeze(1)

            roadmap_state, reward, done, info = envs.step(action_index)
            if info["episodic_outcome"] is not None:
                success_rate = 100 * info['outcome_statistic']['success'] / (episode_ita - episode_start + 1)
                runtime_list.append(info['episodic_avg_runtime'])
                if info["episodic_outcome"] == "success" or info["episodic_outcome"] == "timeout":
                    time_cost_list.append(info['episodic_time_cost'])
                    dist_cost_list.append(info['episodic_dist_cost'])
                
                if len(time_cost_list) > 0 and len(dist_cost_list) > 0:
                    avg_time_cost = sum(time_cost_list) / len(time_cost_list)
                    avg_dist_cost = sum(dist_cost_list) / len(dist_cost_list)
                    std_time_cost = (sum([(tc - avg_time_cost) ** 2 for tc in time_cost_list]) / len(time_cost_list)) ** 0.5
                    std_dist_cost = (sum([(dc - avg_dist_cost) ** 2 for dc in dist_cost_list]) / len(dist_cost_list)) ** 0.5
                    
                print(f"[Evaluation Info]: episode={episode_ita}, "
                      f"outcome={info['episodic_outcome']}, "
                      f"episodic_return={info['episodic_return']:.2f}, \n"
                      f"episodic_length={info['episodic_length']}, \n"
                      f"episodic time cost={info['episodic_time_cost']:.3f} s, \n"
                      f"episodic distance cost={info['episodic_dist_cost']:.3f} m, \n"
                      f"average algorithm runtime={info['episodic_avg_runtime']} s, \n"
                      f"success: {info['outcome_statistic']['success']}, "
                      f"collision: {info['outcome_statistic']['collision']}, "
                      f"timeout: {info['outcome_statistic']['timeout']}, "
                      f"success rate: {success_rate:.1f}% \n"
                      f"average time cost: {avg_time_cost:.3f} ± {std_time_cost:.3f} s, \n"
                      f"average distance cost: {avg_dist_cost:.3f} ± {std_dist_cost:.3f} m, \n")
                
                record_nav_metrics(
                    csv_file=metrics_csv_file, 
                    episode_ita=episode_ita,
                    outcome=info['episodic_outcome'], 
                    episodic_time_cost=info['episodic_time_cost'], 
                    episodic_dist_cost=info['episodic_dist_cost'], 
                    success_rate=success_rate
                )
                
                episode_ita += 1
                if episode_ita < args.eval_num_episodes:
                    print(80 * "*")
                    print(f"  Episode {episode_ita}: ")
                    print(80 * "*")
                break

    evaluation_period = time.time() - start_time
    hours = int(evaluation_period // 3600)
    minutes = int((evaluation_period % 3600) // 60)
    seconds = int(evaluation_period % 60)    
    print(f"Total evaluation time: {hours} h, {minutes} mins, {seconds} s")
    print(f"Success rate: {success_rate:.1f}%")
    print(f"Average episodic time cost: {avg_time_cost:.3f} ± {std_time_cost:.3f}")
    print(f"Average episodic traveling distance: {avg_dist_cost:.3f} ± {std_dist_cost:.3f}")
    envs.close()


if __name__ == "__main__":
    main()
