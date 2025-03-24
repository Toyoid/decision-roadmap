import subprocess
import time
import os
from os import path as os_path
from datetime import datetime


def get_time_str():
    date_time = datetime.fromtimestamp(time.time())
    formatted_date_time = date_time.strftime('%Y%m%d%H%M%S')
    return formatted_date_time


# parameters
run_new_results = True
start_episode = 1
end_episode = 2
map_name = "maze_large"
episode_max_period = 150
goal_near_th = 0.3
ros_node_name = "far_planner_eval_node"

# file paths
current_dir = os_path.dirname(os_path.abspath(__file__))
eval_far_path = current_dir + "/eval_far_planner.py"
save_metrics_dir = current_dir + '/far_planner_metrics'
try:
    if not os.path.exists(save_metrics_dir):
        os.makedirs(save_metrics_dir, exist_ok=True)
except OSError as e:
    print(f"Error while creating directory: {e}")

if run_new_results:
    save_metrics_file = save_metrics_dir + f'/{map_name}__episode-{start_episode}-{end_episode}__{get_time_str()}.txt'
    # multiple evaluation of far-planner
    for i in range(start_episode, end_episode + 1):
        episode_ita = i
        print("\n*****************************************************************************************************")
        print(f"         FAR-Planner Evaluation with Episode {episode_ita}")
        print("*****************************************************************************************************")

        subprocess.run([
            'python3', eval_far_path,
            str(episode_ita),
            '--start_episode', str(start_episode),
            '--end_episode', str(end_episode),
            '--map_name', map_name,
            '--episode_max_period', str(episode_max_period),
            '--goal_near_th', str(goal_near_th),
            '--ros_node_name', ros_node_name,
            '--save_metrics_file', save_metrics_file
        ])

        print("*****************************************************************************************************\n")
        time.sleep(1.5)

    print("\nAll iterations completed.")
else:
    save_metrics_file = save_metrics_dir + "/maze_medium__episode-0-2__20240826123743.txt"

# eval results
time_cost_list = []
dist_cost_list = []
runtime_list = []
outcome_list = []

with open(save_metrics_file, 'r') as infile:
    lines = infile.readlines()

print()
for i, line in enumerate(lines, start=1):
    parts = line.strip().split()
    if len(parts) == 4:
        outcome, episodic_time_cost, episodic_dist_cost, episodic_avg_runtime = parts
        outcome_list.append(int(outcome))
        time_cost_list.append(float(episodic_time_cost))
        dist_cost_list.append(float(episodic_dist_cost))
        runtime_list.append(float(episodic_avg_runtime))
        print(f"Iteration {i} Results:")
        print(f"Outcome: {outcome}")
        print(f"Episodic time cost: {episodic_time_cost} s")
        print(f"Episodic distance cost: {episodic_dist_cost} m")
        print(f"Average algorithm runtime: {episodic_avg_runtime} s")
        print()
    else:
        print(f"Line {i} in the results file is malformed.")

print("\n*****************************************************************************************************")
print(f"  FAR-Planner Evaluation Results ")
print("*****************************************************************************************************")
print(f"Success Rate: {(100 * sum(outcome_list) / len(outcome_list)):.1f}% \n"
      f"Average Episodic Time Cost: {sum(time_cost_list) / len(time_cost_list)} s \n"
      f"Average Episodic Traveling Distance: {sum(dist_cost_list) / len(dist_cost_list)} m \n"
      f"Average Algorithm Runtime: {sum(runtime_list) / len(runtime_list)} s")
print("*****************************************************************************************************\n")
print("All saved results processed.")