import math
import numpy as np
from copy import deepcopy
import torch
import rospy
from envs.nav_drm_env_base import NavDRMEnvBase
from geometry_msgs.msg import Point, PointStamped


class NavFrontierGuidedDRMEnv(NavDRMEnvBase):
    """
    RL Environment Class for Single-Robot Navigation based on Frontier-Guided Decision Roadmap
    """
    def __init__(self, args, robot_namespace, drm_name, is_train=False):
        super().__init__(args, robot_namespace, drm_name, is_train)
        self.roadmap_type = "frontier_guide"

    def step(self, action_index):
        """
        Step funtion of single robot navigation environment based Frontier-guided Decision Roadmap
        """
        assert self.target_position is not None
        assert self.ita_in_episode is not None
        self.ita_in_episode += 1

        '''
        action decode
        '''
        selected_node_idx = self.edge_inputs[0, 0, action_index.item()]  # tensor(scalar)
        selected_node_pos = self.node_coords[selected_node_idx]  # np.array(2,)

        rospy.wait_for_service(self.unpause_gazebo_service_name)
        try:
            self.unpause_gazebo()
        except rospy.ServiceException as e:
            print("Unpause Service Failed: %s" % e)
        '''
        Give action to robot and let robot execute, then get next observation
        '''
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.point.x = selected_node_pos[0]
        goal.point.y = selected_node_pos[1]
        goal.point.z = 1.0
        self.current_goal_pub.publish(goal)

        robot_position = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z]
        execute_start_time = rospy.get_time()
        curr_goal_arrive_flag = False
        step_time_flag = False
        target_reach_flag = False
        while not (curr_goal_arrive_flag or step_time_flag or target_reach_flag or rospy.is_shutdown()):
            dis = math.sqrt((self.odom.pose.pose.position.x - goal.point.x) ** 2 +
                            (self.odom.pose.pose.position.y - goal.point.y) ** 2 +
                            (self.odom.pose.pose.position.z - goal.point.z) ** 2)
            curr_goal_arrive_flag = dis <= 0.9
            step_time_flag = (rospy.get_time() - execute_start_time) >= self.step_time

            target_dis = math.sqrt((self.odom.pose.pose.position.x - self.target_position[0]) ** 2 +
                                   (self.odom.pose.pose.position.y - self.target_position[1]) ** 2 +
                                   (self.odom.pose.pose.position.z - self.target_position[2]) ** 2)
            target_reach_flag = (target_dis <= self.goal_near_th)

        # check whether local planner get stuck
        robot_move = math.sqrt((self.odom.pose.pose.position.x - robot_position[0]) ** 2 +
                               (self.odom.pose.pose.position.y - robot_position[1]) ** 2 +
                               (self.odom.pose.pose.position.z - robot_position[2]) ** 2)
        if robot_move <= 0.025:
            rospy.wait_for_service(self.escape_stuck_service_name)
            try:
                resp = self.escape_stuck()
            except rospy.ServiceException as e:
                print("Escape Stuck Service Failed: %s" % e)
            print("[ROS Service Request]: adjusting robot to escape stuck...")

        # get next state
        robot_pose, roadmap_state = self._get_next_state()
        '''
        Then pause the simulation
        1. Compute rewards of the actions
        2. Check if the episode is ended
        '''
        rospy.wait_for_service(self.pause_gazebo_service_name)
        try:
            self.pause_gazebo()
        except rospy.ServiceException as e:
            print("Pause Service Failed: %s" % e)

        if target_reach_flag:
            reward = self.goal_reward + self.step_penalty
            done = True
            self.info["episodic_time_cost"] = self.time_duration - self.episodic_start_time
            self.info["episodic_dist_cost"] = self.traveling_distance - self.episodic_start_dist
            self.info["episodic_avg_runtime"] = self.avg_runtime
            self.info["episodic_outcome"] = "success"
            self.info["outcome_statistic"]["success"] += 1
            print("[Episodic Outcome]: Goal achieved!")
        else:
            target_dis, target_dir = self._compute_dis_dir_2_goal(robot_pose)
            self.target_dis_dir_cur = [target_dis, target_dir]
            reward, done = self._compute_reward(robot_pose)
            self.target_dis_dir_pre = [self.target_dis_dir_cur[0], self.target_dis_dir_cur[1]]

        self.info["episodic_return"] += reward
        self.info["episodic_length"] += 1

        reward = torch.tensor(reward).view(1, 1, 1).to(self.device)
        done = torch.tensor(done, dtype=torch.int32).view(1, 1, 1).to(self.device)

        return roadmap_state, reward, done, self.info
    
    def _get_next_state(self):
        # Get Robot Pose
        # compute yaw from quaternion
        quat = [self.odom.pose.pose.orientation.x,
                self.odom.pose.pose.orientation.y,
                self.odom.pose.pose.orientation.z,
                self.odom.pose.pose.orientation.w]
        siny_cosp = 2. * (quat[0] * quat[1] + quat[2] * quat[3])
        cosy_cosp = 1. - 2. * (quat[1] ** 2 + quat[2] ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)  # range from -pi to pi
        robot_pose = np.array([self.odom.pose.pose.position.x,
                               self.odom.pose.pose.position.y,
                               self.odom.pose.pose.position.z, yaw])

        # Get Roadmap State
        '''
        Required Inputs:
        node_inputs, edge_inputs, current_index, node_padding_mask, edge_padding_mask, edge_mask

        Need to Provide:
            (x, y)
            utility
            frontier_cost
            (dx, dy, dis) -> node_inputs, node_padding_mask

            current_node_index (scalar: idx) -> current_index (tensor: (1,1,1))

            edges -> edges_inputs, edge_padding_mask, edge_mask
        '''
        rospy.wait_for_service(self.get_roadmap_service_name)
        try:
            roadmap_resp = self.get_roadmap()
        except rospy.ServiceException as e:
            print("Get Roadmap Service Failed: %s" % e)

        # 1. acquire node position, node utility, edges
        node_pos = []
        node_utility = []
        frontier_cost = []
        edges_adj_pos = []  # (num_nodes, adjacent_node_positions)
        edges_adj_idx = []  # (num_nodes, adjacent_node_indexes)
        for marker in roadmap_resp.roadmapMarkers.markers:
            if marker.ns == f'{self.roadmap_type}/prm_point':
                pos = [marker.pose.position.x, marker.pose.position.y]
                node_id = marker.id
                node_pos.append(pos)
            elif marker.ns == f'{self.roadmap_type}/num_voxel_text':
                assert marker.id == node_id, "Utility does not match with PRM node"
                node_utility.append(float(marker.text))
            # elif marker.ns == f'{self.roadmap_type}/is_nav_target':
            #     assert marker.id == node_id, f"Nav-target flag does not match with PRM node: flag_marker_id({marker.id}) != node_id({node_id})"
            #     assert [marker.pose.position.x, marker.pose.position.y] == pos, "Nav-target flag position does not match with PRM node"
            #     self.nav_target_found = True
            elif marker.ns == f'{self.roadmap_type}/path_cost_text':
                assert marker.id == node_id, "Path-Cost does not match with PRM node"
                cost_str = marker.text.split(':')[1]
                cost = float(cost_str) if cost_str != 'inf' else -1.0
                frontier_cost.append(cost)
            elif marker.ns == f'{self.roadmap_type}/edge':
                edges_from_current_node = [marker.points[0]]
                for i, p in enumerate(marker.points):
                    if (i % 2) == 1:
                        edges_from_current_node.append(p)
                edges_adj_pos.append(edges_from_current_node)

        # Add robot position as a new node
        robot_pos = [robot_pose[0], robot_pose[1]]
        node_pos.append(robot_pos)
        node_utility.append(0.0)  # utility of Robot node is 0
        frontier_cost.append(-1.0)  # frontier cost of Robot node is -1

        # Create edges from robot node to all other nodes
        robot_edges = [Point(x=robot_pos[0], y=robot_pos[1], z=1.0)] # start point of the edge
        for pos in node_pos[:-1]:  # Exclude the robot node itself
            robot_edges.append(Point(x=pos[0], y=pos[1], z=1.0))  # end point of the edge
        edges_adj_pos.append(robot_edges)

        # 2. compute direction_vector
        assert len(node_pos) > 0, "PRM node list is empty"
        assert len(node_pos) == len(node_utility), "Node position and utility size mismatch"
        self.node_coords = np.round(node_pos, 3)
        n_nodes = self.node_coords.shape[0]
        dis_vector = [self.target_position[0], self.target_position[1]] - self.node_coords
        dis = np.sqrt(np.sum(dis_vector**2, axis=1))
        # normalize direction vector
        non_zero_indices = dis != 0
        dis = np.expand_dims(dis, axis=1)
        dis_vector[non_zero_indices] = dis_vector[non_zero_indices] / dis[non_zero_indices]
        dis /= self.coords_norm_coef
        direction_vector = np.concatenate((dis_vector, dis), axis=1)

        # 3. normalize node attributes
        # normalize node coordinates
        node_coords_norm = self.node_coords / self.coords_norm_coef

        # normalize node utility
        node_utility_inputs = np.array(node_utility).reshape(n_nodes, 1)
        node_utility_inputs = (node_utility_inputs - node_utility_inputs.min()) / (node_utility_inputs.max() - node_utility_inputs.min() + 1e-4)

        # normalize frontier cost        
        frontier_cost_inputs = np.array(frontier_cost).reshape(n_nodes, 1)
        valid_mask = frontier_cost_inputs >= 0  # -1 for unreachable/invalid frontier ndoes
        if np.any(valid_mask):
            valid_costs = frontier_cost_inputs[valid_mask]
            min_cost = np.min(valid_costs)
            max_cost = np.max(valid_costs)
            if max_cost > min_cost:  # Avoid division by zero
                # Normalize valid costs to [0, 1] range
                frontier_cost_inputs[valid_mask] = ((valid_costs - min_cost) / (max_cost - min_cost)) + 1e-2    
            else:
                frontier_cost_inputs[valid_mask] = 1e-2  # If all costs are the same
        
        # 4. formulate node_inputs tensor
        node_inputs = np.concatenate((node_coords_norm, node_utility_inputs, frontier_cost_inputs, direction_vector), axis=1)
        node_inputs = torch.FloatTensor(node_inputs).unsqueeze(0).to(self.device)

        # 5. calculate a mask for padded node
        if self.is_train:
            assert n_nodes <= self.node_padding_size, f"Number of nodes: {n_nodes}"
            padding = torch.nn.ZeroPad2d((0, 0, 0, self.node_padding_size - n_nodes))
            node_inputs = padding(node_inputs)
            # calculate a mask to padded nodes
            node_padding_mask = torch.zeros((1, 1, n_nodes), dtype=torch.int64).to(self.device)
            node_padding = torch.ones((1, 1, self.node_padding_size - n_nodes), dtype=torch.int64).to(self.device)
            node_padding_mask = torch.cat((node_padding_mask, node_padding), dim=-1)

            # print(f"node_inputs: {node_inputs.shape}\n{node_inputs}\n")
            # print(f"node_padding_mask: {node_padding_mask.shape}\n{node_padding_mask}\n")
        else:
            node_padding_mask = None

        # 6. compute current node index
        current_node_idx = n_nodes - 1  # robot node is the last node
        current_index = torch.tensor([current_node_idx]).unsqueeze(0).unsqueeze(0).to(self.device)

        # 7. compute edge_inputs
        for adj_nodes in edges_adj_pos:
            adj_node_idxs = []
            for adj_node_pos in adj_nodes:
                pos = np.round([adj_node_pos.x, adj_node_pos.y], 3)
                assert len(pos) == self.node_coords.shape[1], "Wrong dimension on node coordinates"
                try:
                    idx = np.argwhere((self.node_coords == pos).all(axis=1)).item()
                except ValueError as e:
                    print("[Error]: Error in finding adjacent node index: %s" % e)
                    print(f"[Error]: idx array: {np.argwhere((self.node_coords == pos).all(axis=1))}")
                    print("[Error]: pos array: %s" % pos)
                    print("[Error]: node_coords array: %s" % self.node_coords)
                except Exception as e:
                    print(f"Unexpected {e}, {type(e)}")
                    raise
                adj_node_idxs.append(idx)
            edges_adj_idx.append(adj_node_idxs)

        adjacent_matrix = self.calculate_edge_mask(edges_adj_idx)  # (n_nodes, n_nodes)
        edge_mask = torch.from_numpy(adjacent_matrix).float().unsqueeze(0).to(self.device)  # (1, )
        # pad edge mask and edge_inputs while training
        if self.is_train:
            # pad edge mask to node_padding_size
            assert n_nodes <= self.node_padding_size
            padding = torch.nn.ConstantPad2d(
                (0, self.node_padding_size - n_nodes, 0, self.node_padding_size - n_nodes), 1)
            edge_mask = padding(edge_mask)

            # pad edge_inputs to k_neighbor_size with -1 (instead of 0)
            edge = deepcopy(edges_adj_idx[current_index])
            assert len(edge) <= self.k_neighbor_size, "Number of connected frontier nodes of robot exceeds k_neighbor_size"
            while len(edge) < self.k_neighbor_size:
                edge.append(-1)  # Using -1 as padding value instead of 0

            self.edge_inputs = torch.tensor(edge).unsqueeze(0).unsqueeze(0).to(self.device)  # (1, 1, k_neighbor_size)

            edge_padding_mask = torch.zeros((1, 1, self.k_neighbor_size), dtype=torch.int64).to(self.device)
            one = torch.ones_like(edge_padding_mask, dtype=torch.int64).to(self.device)
            edge_padding_mask = torch.where(self.edge_inputs == -1, one, edge_padding_mask)
        
            zero = torch.zeros_like(self.edge_inputs, dtype=torch.int64).to(self.device)
            self.edge_inputs = torch.where(self.edge_inputs == -1, zero, self.edge_inputs) # replace -1 with 0, since -1 is invalid for later use in attention_networks.py

            # print(f"edge_inputs: {self.edge_inputs.shape}\n{self.edge_inputs}\n")
            # print(f"edge_padding_mask: {edge_padding_mask.shape}\n{edge_padding_mask}\n")
            # print(f"edge_mask: {edge_mask.shape}\n{edge_mask}\n")
        else:
            # no need for padding if not training, as robot node is connected to all other frontier nodes
            edge = deepcopy(edges_adj_idx[current_index])
            self.edge_inputs = torch.tensor(edge).unsqueeze(0).unsqueeze(0).to(self.device)  # (1, 1, num_frontier_node)
            edge_padding_mask = torch.zeros_like(self.edge_inputs, dtype=torch.int64).to(self.device)

        # 8. update roadmap state
        roadmap_state = node_inputs, self.edge_inputs, current_index, node_padding_mask, edge_padding_mask, edge_mask

        return robot_pose, roadmap_state
    
