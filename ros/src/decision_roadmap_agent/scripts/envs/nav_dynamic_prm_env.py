import math
import numpy as np
from scipy.spatial import KDTree
import torch
import rospy
from envs.nav_drm_env_base import NavDRMEnvBase
from geometry_msgs.msg import PointStamped


class NavDynamicPRMEnv(NavDRMEnvBase):
    """
    RL Environment Class for Single-Robot Navigation based on Dynamic PRM Decision Roadmap
    """
    def step(self, action_index):
        """
        Step funtion of single robot navigation environment based on decision roadmap
        """
        assert self.target_position is not None
        assert self.ita_in_episode is not None
        self.ita_in_episode += 1

        '''
        action decode
        '''
        if self.nav_target_found:
            selected_node_pos = self.target_position[:2]
        else:
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

        # update route_node for computing guidepost
        if curr_goal_arrive_flag:
            if self.route_node.shape[0] > 0:
                x = selected_node_pos[0] + selected_node_pos[1] * 1j
                node_complex = self.route_node[:, 0] + self.route_node[:, 1] * 1j
                distances = np.abs(x - node_complex)
                # assume a node has been visited when query distance is lower than 0.8
                if np.any(distances <= 0.8):
                    self.is_revisiting = True
                    print(f"[DRM]: Revisiting node at: [{selected_node_pos[0]}, {selected_node_pos[1]}]")
                    print(f"[DRM]: Route node size: {self.route_node.shape[0]}")
                else:
                    self.route_node = np.append(self.route_node, [selected_node_pos], axis=0)
            else:
                self.route_node = np.array([selected_node_pos])

        # check whether local planner get stuck
        robot_move = math.sqrt((self.odom.pose.pose.position.x - robot_position[0]) ** 2 +
                               (self.odom.pose.pose.position.y - robot_position[1]) ** 2 +
                               (self.odom.pose.pose.position.z - robot_position[2]) ** 2)
        if robot_move <= 0.04:
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
            guidepost
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
        edges_adj_pos = []  # (num_nodes, adjacent_node_positions)
        edges_adj_idx = []  # (num_nodes, adjacent_node_indexes)
        for marker in roadmap_resp.roadmapMarkers.markers:
            if marker.ns == 'prm_point':
                # pos = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
                pos = [marker.pose.position.x, marker.pose.position.y]
                node_id = marker.id
                node_pos.append(pos)
            elif marker.ns == 'num_voxel_text':
                assert len(node_pos) > 0, "PRM node list is empty"
                utility_match = marker.id == node_id
                assert utility_match, "Utility does not match with PRM node"
                node_utility.append(float(marker.text))
            elif marker.ns == 'edge':
                edges_from_current_node = [marker.points[0]]
                for i, p in enumerate(marker.points):
                    if (i % 2) == 1:
                        edges_from_current_node.append(p)
                edges_adj_pos.append(edges_from_current_node)

        # 2. compute direction_vector
        self.node_coords = np.round(node_pos, 3)
        n_nodes = self.node_coords.shape[0]
        dis_vector = [self.target_position[0], self.target_position[1]] - self.node_coords
        dis = np.sqrt(np.sum(dis_vector**2, axis=1))
        if any(dis <= 0.001):
            self.nav_target_found = True
        # normalize direction vector
        non_zero_indices = dis != 0
        dis = np.expand_dims(dis, axis=1)
        dis_vector[non_zero_indices] = dis_vector[non_zero_indices] / dis[non_zero_indices]
        dis /= self.coords_norm_coef
        direction_vector = np.concatenate((dis_vector, dis), axis=1)

        # 3. compute guidepost
        guidepost = np.zeros((n_nodes, 1))
        x = self.node_coords[:, 0] + self.node_coords[:, 1] * 1j
        for node in self.route_node:
            node_complex = node[0] + node[1] * 1j
            distances = np.abs(x - node_complex)
            # assume a node has been visited when query distance is lower than 0.9 (PRM sampling dist threshold: 0.8)
            index = np.argwhere(distances < 0.9)
            guidepost[index] = 1

        # 4. formulate node_inputs tensor
        # normalize node observations
        node_coords_norm = self.node_coords / self.coords_norm_coef
        node_utility_inputs = np.array(node_utility).reshape(n_nodes, 1)
        node_utility_inputs = node_utility_inputs / self.utility_norm_coef
        # concatenate
        node_inputs = np.concatenate((node_coords_norm, node_utility_inputs, guidepost, direction_vector), axis=1)
        node_inputs = torch.FloatTensor(node_inputs).unsqueeze(0).to(self.device)

        # 5. calculate a mask for padded node
        if self.is_train:
            assert n_nodes < self.node_padding_size, f"Number of nodes: {n_nodes}"
            padding = torch.nn.ZeroPad2d((0, 0, 0, self.node_padding_size - n_nodes))
            node_inputs = padding(node_inputs)
            # calculate a mask to padded nodes
            node_padding_mask = torch.zeros((1, 1, n_nodes), dtype=torch.int64).to(self.device)
            node_padding = torch.ones((1, 1, self.node_padding_size - n_nodes), dtype=torch.int64).to(self.device)
            node_padding_mask = torch.cat((node_padding_mask, node_padding), dim=-1)
        else:
            node_padding_mask = None

        # 6. compute current node index
        current_node_idx = np.argmin(np.linalg.norm(self.node_coords - robot_pose[:2], axis=1))
        current_index = torch.tensor([current_node_idx]).unsqueeze(0).unsqueeze(0).to(self.device)

        # 7. compute edge_inputs
        for adj_nodes in edges_adj_pos:
            adj_node_idxs = []
            for adj_node_pos in adj_nodes:
                # pos = [adj_node_pos.x, adj_node_pos.y, adj_node_pos.z]
                pos = np.round([adj_node_pos.x, adj_node_pos.y], 3)
                assert len(pos) == self.node_coords.shape[1], "Wrong dimension on node coordinates"
                try:
                    idx = np.argwhere((self.node_coords == pos).all(axis=1)).item()
                except ValueError as e:
                    print("[Error]: Error in finding adjacent node index: %s" % e)
                    print(f"[Error]: idx array: {np.argwhere((self.node_coords == pos).all(axis=1))}")
                except Exception as e:
                    print(f"Unexpected {e}, {type(e)}")
                    raise
                adj_node_idxs.append(idx)
            edges_adj_idx.append(adj_node_idxs)

        adjacent_matrix = self.calculate_edge_mask(edges_adj_idx)
        edge_mask = torch.from_numpy(adjacent_matrix).float().unsqueeze(0).to(self.device)
        # pad edge mask to node_padding_size while training
        if self.is_train:
            assert n_nodes < self.node_padding_size
            padding = torch.nn.ConstantPad2d(
                (0, self.node_padding_size - n_nodes, 0, self.node_padding_size - n_nodes), 1)
            edge_mask = padding(edge_mask)

        # pad edge_inputs with k-nearest neighbors
        current_node_pos = self.node_coords[current_node_idx]
        
        '''action space option 1: k-nearest neighbors'''
        # # build kd-tree to search k-nearest neighbors
        # kdtree = KDTree(self.node_coords)
        # k = min(self.k_neighbor_size, n_nodes)
        # _, nearest_indices = kdtree.query(current_node_pos, k=k)

        # # # padding option 1: pad edge_inputs to k_nearest_size with 0, this will filter node_0 as unconnected node
        # # edge = np.pad(nearest_indices, (0, self.k_neighbor_size - k), mode='constant', constant_values=0)
        # # self.edge_inputs = torch.tensor(edge).unsqueeze(0).unsqueeze(0).to(self.device)  # (1, 1, k_neighbor_size)
        # #
        # # edge_padding_mask = torch.zeros((1, 1, self.k_neighbor_size), dtype=torch.int64).to(self.device)
        # # one = torch.ones_like(edge_padding_mask, dtype=torch.int64).to(self.device)
        # # edge_padding_mask = torch.where(self.edge_inputs == 0, one, edge_padding_mask)

        # # padding option 2: pad edge_inputs to k_nearest_size with -1 first, this keeps node_0 if it is connected
        # edge = np.pad(nearest_indices, (0, self.k_neighbor_size - k), mode='constant', constant_values=-1)
        # self.edge_inputs = torch.tensor(edge).unsqueeze(0).unsqueeze(0).to(self.device)  # (1, 1, k_neighbor_size)

        '''action space option 2: all adjacent nodes'''
        edge = np.array(edges_adj_idx[current_node_idx])
        k = min(len(edge), self.k_neighbor_size)
        if len(edge) > self.k_neighbor_size:
            print(f"[DRM]: edge size = {len(edge)}, k neighbor size = {self.k_neighbor_size}")
        edge = np.pad(edge, (0, self.k_neighbor_size - k), mode='constant', constant_values=-1)
        self.edge_inputs = torch.tensor(edge).unsqueeze(0).unsqueeze(0).to(self.device)  # (1, 1, k_neighbor_size)
        
        '''construct edge mask'''
        edge_padding_mask = torch.zeros((1, 1, self.k_neighbor_size), dtype=torch.int64).to(self.device)
        one = torch.ones_like(edge_padding_mask, dtype=torch.int64).to(self.device)
        edge_padding_mask = torch.where(self.edge_inputs == -1, one, edge_padding_mask)
        zero = torch.zeros_like(edge_padding_mask, dtype=torch.int64).to(self.device)
        self.edge_inputs = torch.where(self.edge_inputs == -1, zero, self.edge_inputs)  # change the unconnected node idxs from -1 back to 0 for attetion network

        # 8. update roadmap state
        roadmap_state = node_inputs, self.edge_inputs, current_index, node_padding_mask, edge_padding_mask, edge_mask

        return robot_pose, roadmap_state
    
    def _compute_reward(self, robot_pose):
        """
        Reward funtion:
        1. R_Arrive If Distance to Goal is smaller than D_goal
        2. R_Collision If Distance to Obstacle is smaller than D_obs
        3. a * (Last step distance to goal - current step distance to goal)
        4. R_Penalty for each step 
        5. R_Revisit If revisiting a node (only for dynamic PRM and dynamic UG)
        """
        episodic_time_cost = self.time_duration - self.episodic_start_time
        episodic_dist_cost = self.traveling_distance - self.episodic_start_dist

        done = False
        if self.target_dis_dir_cur[0] < self.goal_near_th:
            reward = self.goal_reward
            done = True
            self.info["episodic_time_cost"] = episodic_time_cost
            self.info["episodic_dist_cost"] = episodic_dist_cost
            self.info["episodic_avg_runtime"] = self.avg_runtime
            self.info["episodic_outcome"] = "success"
            self.info["outcome_statistic"]["success"] += 1
            print("[Episodic Outcome]: Goal achieved!")
        # elif self.robot_collision:
        #     reward = self.collision_reward
        #     done = True
        #     self.info["episodic_time_cost"] = episodic_time_cost
        #     self.info["episodic_dist_cost"] = episodic_dist_cost
        #     self.info["episodic_avg_runtime"] = self.avg_runtime
        #     self.info["episodic_outcome"] = "collision"
        #     self.info["outcome_statistic"]["collision"] += 1
        #     print("[Episodic Outcome]: Collides with obstacles!")
        elif (robot_pose[2] <= self.env_height_range[0]) or (robot_pose[2] >= self.env_height_range[1]):
            reward = self.collision_reward
            done = True
            self.info["episodic_time_cost"] = episodic_time_cost
            self.info["episodic_dist_cost"] = episodic_dist_cost
            self.info["episodic_avg_runtime"] = self.avg_runtime
            self.info["episodic_outcome"] = "collision"
            self.info["outcome_statistic"]["collision"] += 1
            print("[Episodic Outcome]: Out of flying range!")
        elif self.ita_in_episode >= self.max_episode_steps:
            reward = self.goal_dis_amp * (self.target_dis_dir_pre[0] - self.target_dis_dir_cur[0])
            done = True
            self.info["episodic_time_cost"] = episodic_time_cost
            self.info["episodic_dist_cost"] = episodic_dist_cost
            self.info["episodic_avg_runtime"] = self.avg_runtime
            self.info["episodic_outcome"] = "timeout"
            self.info["outcome_statistic"]["timeout"] += 1
            print("[Episodic Outcome]: Navigation timeout!")
        else:
            reward = self.goal_dis_amp * (self.target_dis_dir_pre[0] - self.target_dis_dir_cur[0])
            if self.is_revisiting:
                reward += self.revisit_penalty
        reward += self.step_penalty

        return reward, done
