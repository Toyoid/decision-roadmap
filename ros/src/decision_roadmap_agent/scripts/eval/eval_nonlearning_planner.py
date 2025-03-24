#!/usr/bin/python3

import time
import rospy
import argparse
from envs.nonlearning_planner_wrapper import NonLearningPlannerWrapper
from utils import record_nav_metrics, get_time_str


def main():
    ros_node_name = "eval_nonlearning_planner"
    rospy.init_node(ros_node_name)

    robot_namespace = rospy.get_param('~robot_ns', '')
    is_assist_nav = rospy.get_param('~is_assist_nav', False)
    map_name = rospy.get_param('~map_name', 'maze_medium')

    eval_num_episodes = 40
    episode_max_period = 124
    goal_near_th = 0.3
    episode_start = 0
    metrics_csv_file = f"dprm_planner_{map_name}_metrics_{get_time_str()}.csv"

    # env setup
    envs = NonLearningPlannerWrapper(
        robot_namespace=robot_namespace,
        is_assist_nav=is_assist_nav,
        episodic_max_period=episode_max_period, 
        goal_near_th=goal_near_th,
        map_name=map_name,
    )

    # eval results
    time_cost_list = []
    dist_cost_list = []
    runtime_list = []
    success_rate = None
    avg_time_cost = None
    avg_dist_cost = None
    std_time_cost = None
    std_dist_cost = None

    # run the experiment
    print(80 * "*")
    print(f"  Robot {robot_namespace} Episode {episode_start}: ")
    print(80 * "*")
    episode_ita = episode_start
    total_start_time = time.time()
    while episode_ita < eval_num_episodes:
        envs.reset(episode_ita)
        while not rospy.is_shutdown():
            info = envs.get_info()
            # check the episode end points and log the relevant episodic return (not considering parallel envs)
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
                    
                print(f"\n[Evaluation Info]: episode={episode_ita}, "
                    f"outcome={info['episodic_outcome']}, \n"
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
                if episode_ita < eval_num_episodes:
                    print(80 * "*")
                    print(f"  Robot {robot_namespace} Episode {episode_ita}: ")
                    print(80 * "*")
                break

    eval_period = time.time() - total_start_time
    hours = int(eval_period // 3600)
    minutes = int((eval_period % 3600) // 60)
    seconds = int(eval_period % 60)
    print(f"Total evaluation time: {hours} h, {minutes} mins, {seconds} s")
    print(f"Success rate: {success_rate:.1f}%")
    print(f"Average episodic time cost: {avg_time_cost:.3f} ± {std_time_cost:.3f}")
    print(f"Average episodic traveling distance: {avg_dist_cost:.3f} ± {std_dist_cost:.3f}")
    envs.close()


if __name__ == "__main__":
    main()
