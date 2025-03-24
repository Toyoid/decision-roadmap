import rospy
import subprocess
from envs.far_planner_wrapper import FARPlannerWrapper
import os
import signal
import argparse
from os import path as os_path
from datetime import datetime
import time


def get_time_str():
    date_time = datetime.fromtimestamp(time.time())
    formatted_date_time = date_time.strftime('%Y%m%d%H%M%S')
    return formatted_date_time


def start_roscore():
    try:
        # try using rosnode ping master to check whether roscore is running
        subprocess.check_call(['rosnode', 'ping', '-c', '1', 'rosout'])
    except subprocess.CalledProcessError:
        # if roscore not running，start roscore
        subprocess.Popen(['roscore'])
        rospy.sleep(2.0)
        print("[ROS Launch]: roscore started...")


def stop_roscore():
    try:
        # Find the process ID of roscore
        roscore_pid = subprocess.check_output(['pgrep', '-f', 'roscore']).decode('utf-8').strip()
        # Send SIGINT (interrupt) signal to terminate the roscore process
        os.kill(int(roscore_pid), signal.SIGINT)
        print("[ROS Shutdown]: roscore stopped...")
    except subprocess.CalledProcessError:
        print("[ROS Shutdown]: roscore is not running.")


def main(args):
    stop_roscore()
    rospy.sleep(2)
    start_roscore()

    rospy.init_node(args.ros_node_name)
    envs = FARPlannerWrapper(ros_node_name=args.ros_node_name, map_name=args.map_name,
                             episodic_max_period=args.episode_max_period, goal_near_th=args.goal_near_th)

    # run the experiment for only once
    envs.reset(args.episode_ita)
    while not rospy.is_shutdown():
        info = envs.get_info()
        # check the episode end points and log the relevant episodic return (not considering parallel envs)
        if info["episodic_outcome"] is not None:
            print(
                "\n*******************************************************************************************")
            print(f"  Episode {args.episode_ita}: ")
            print(
                "*******************************************************************************************")
            print(f"[Evaluation Info]: episode={args.episode_ita}, "
                  f"outcome={info['episodic_outcome']}, \n"
                  f"episodic time cost={info['episodic_time_cost']} s, \n"
                  f"episodic distance cost={info['episodic_dist_cost']} m, \n"
                  f"average algorithm runtime={info['episodic_avg_runtime']} s, \n"
                  f"success: {info['outcome_statistic']['success']}, "
                  f"collision: {info['outcome_statistic']['collision']}, "
                  f"timeout: {info['outcome_statistic']['timeout']} \n")

            # save navigation results in file
            outcome = 1 if info['episodic_outcome'] == "success" else 0
            try:
                data_line = f"{outcome} {info['episodic_time_cost']} {info['episodic_dist_cost']} {info['episodic_avg_runtime']}\n"
                with open(args.save_metrics_file, 'a') as outfile:
                    outfile.write(data_line)
                print(f"Results successfully saved to {args.save_metrics_file}")
            except OSError as e:
                print(f"Error while writing file: {e}")
            print("\n*******************************************************************************************\n")

            envs.end_far_planner()
            break

    stop_roscore()


if __name__ == "__main__":
    current_dir = os_path.dirname(os_path.abspath(__file__))
    save_metrics_dir = current_dir + '/far_planner_metrics'
    try:
        if not os.path.exists(save_metrics_dir):
            os.makedirs(save_metrics_dir, exist_ok=True)
    except OSError as e:
        print(f"Error while creating directory: {e}")

    metrics_file = save_metrics_dir + f'/maze_medium__episode-0-0__{get_time_str()}.txt'

    parser = argparse.ArgumentParser(description="Evaluate FAR Planner with 'episode_ita'")
    parser.add_argument('episode_ita', type=int, default=0, help="Set start-goal locations of the episode according to episode_ita")
    parser.add_argument('--start_episode', type=int, default=0, help="The start episode to evaluate far-planner")
    parser.add_argument('--end_episode', type=int, default=0, help="The end episode to evaluate far-planner")
    parser.add_argument('--map_name', type=str, default="maze_medium", help="The name of the world map")
    parser.add_argument('--episode_max_period', type=int, default=150, help="The maximum navigation time duration for the episode")
    parser.add_argument('--goal_near_th', type=float, default=0.3, help="The goal-reach threshold")
    parser.add_argument('--ros_node_name', type=str, default="far_planner_eval_node", help="The name of the eval node")
    parser.add_argument('--save_metrics_file', type=str, default=metrics_file, help="File path to save evaluation metrics results")

    args = parser.parse_args()

    main(args)
