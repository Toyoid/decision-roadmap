import pickle
import random
import numpy as np
from matplotlib import pyplot as plt
from copy import deepcopy


def sample_one_robot_target_pair(all_positions, robot_target_diff):
    """
    Randomly generate one pair of robot pose and target
    """
    target = random.choice(all_positions)
    pose = random.choice(all_positions)
    dist = np.hypot(target[0] - pose[0], target[1] - pose[1])
    while dist < robot_target_diff:
        pose = random.choice(all_positions)
        dist = np.hypot(target[0] - pose[0], target[1] - pose[1])
    pose.append(random.random() * 2 * np.pi)
    return pose, target


def gen_robot_target_pairs(candidate_vertex_list, num_pairs, robot_target_diff=38):
    """
    Generate initial robot-target pairs for RL training and testing
    :return: robot-target pairs
    """
    robot_pose_list = []
    target_pos_list = []
    for i in range(num_pairs):
        init_robot_pose, init_target_pos = sample_one_robot_target_pair(deepcopy(candidate_vertex_list), robot_target_diff)  # deepcopy is critical
        robot_pose_list.append(init_robot_pose)
        target_pos_list.append(init_target_pos)
    robot_target_pair_list = [robot_pose_list, target_pos_list]
    return robot_target_pair_list


if __name__ == "__main__":
    from os import path as os_path

    world_name = "maze_large"
    current_dir = os_path.dirname(os_path.abspath(__file__))
    traj_file_path = current_dir + "/random_positions/" + world_name + "/trajectory" + ".txt"

    print("Loading trajectory.txt...")

    with open(traj_file_path, 'r') as file:
        lines = file.readlines()

    candidate_point_list = []
    for line in lines:
        values = line.split()
        point = [float(values[0]), float(values[1]), float(values[2])]
        candidate_point_list.append(point)
    print("trajectory.txt data loaded.")
    candidate_point_list.pop(0)
    print(f"Number of Vertices: {len(candidate_point_list)}\n"
          f"Vertice Dimension: {len(candidate_point_list[0])}")

    roadmap_file_path = current_dir + "/random_positions/" + world_name + "/roadmap_vertices" + ".p"
    f = open(roadmap_file_path, 'wb')
    pickle.dump(candidate_point_list, f)
    f.close()
    print("\n\nRoadmap vertices saved.")

    # sample robot-target pairs
    # robot_target_diff: 38 (indoor), 9 (maze_medium), 9 (maze_large: x=[-10,10], y=[-15,15])
    robot_target_pairs = gen_robot_target_pairs(candidate_point_list, num_pairs=1000, robot_target_diff=9)
    # save robot-target pairs
    file_path = current_dir + "/random_positions/" + world_name + "/robot_target_poses" + ".p"
    f = open(file_path, 'wb')
    pickle.dump(robot_target_pairs, f)
    f.close()
    print("Robot-target pairs saved.")

    # plot for test
    f = open(file_path, 'rb')
    read_robot_target_pairs = pickle.load(f)
    f.close()
    robot_poses, target_positions = read_robot_target_pairs

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')  # make width = height in each subplot
    ax.scatter([p[0] for p in robot_poses], [p[1] for p in robot_poses], c='darkcyan', s=15, alpha=0.5, zorder=2)
    ax.scatter([p[0] for p in target_positions], [p[1] for p in target_positions], c='r', alpha=0.5, zorder=1)
    for i in range(len(target_positions)):
        ax.plot([robot_poses[i][0], target_positions[i][0]], [robot_poses[i][1], target_positions[i][1]], linewidth=1.5,
                c='lightgrey', linestyle="--", zorder=0)
    plt.show()

