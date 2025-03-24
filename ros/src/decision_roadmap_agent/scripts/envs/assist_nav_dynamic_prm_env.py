import math
import numpy as np
from copy import deepcopy
import torch
import rospy
from envs.assist_nav_env_base import AssistNavEnvBase
from geometry_msgs.msg import Point


class AssistNavDynamicPRMEnv(AssistNavEnvBase):
    """
    RL Environment Class for Single-Robot Navigation based on Dynamic PRM Decision Roadmap
    """
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
        targets = np.array(list(self.target_positions.values()), dtype=np.float64)  # (n_targets, 2)
        direction_vectors = self._compute_direction_vectors(self.node_coords, targets, self.coords_norm_coef)  # (n_nodes, n_targets*3)

        # 3. compute human_intention_vector
        other_robot_pos = np.stack((self.other_robot_pos_pre, self.other_robot_pos_cur), axis=0)  # (2, 2)
        intent_vectors = self._compute_intention_vectors(other_robot_pos, targets, self.coords_norm_coef)  # (1, n_targets*6)
        intent_vectors = np.tile(intent_vectors, (n_nodes, 1))  # (n_nodes, n_targets*6)

        # 4. normalize node attributes
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
        
        # 5. formulate node_inputs tensor
        node_inputs = np.concatenate((node_coords_norm, node_utility_inputs, frontier_cost_inputs, direction_vectors, intent_vectors), axis=1)
        node_inputs = torch.FloatTensor(node_inputs).unsqueeze(0).to(self.device)

        # 6. calculate a mask for padded node
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

        # 7. compute current node index
        current_node_idx = n_nodes - 1  # robot node is the last node
        current_index = torch.tensor([current_node_idx]).unsqueeze(0).unsqueeze(0).to(self.device)

        # 8. compute edge_inputs
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

        adjacent_matrix = self.calculate_edge_mask(edges_adj_idx)
        edge_mask = torch.from_numpy(adjacent_matrix).float().unsqueeze(0).to(self.device)
        # pad edge mask and edge_inputs while training
        if self.is_train:
            # pad edge mask to node_padding_size
            assert n_nodes < self.node_padding_size
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
        else:
            # no need for padding if not training, as robot node is connected to all other frontier nodes
            edge = deepcopy(edges_adj_idx[current_index])
            self.edge_inputs = torch.tensor(edge).unsqueeze(0).unsqueeze(0).to(self.device)  # (1, 1, num_frontier_node)
            edge_padding_mask = torch.zeros_like(self.edge_inputs, dtype=torch.int64).to(self.device)

        # 9. update roadmap state
        roadmap_state = node_inputs, self.edge_inputs, current_index, node_padding_mask, edge_padding_mask, edge_mask

        return robot_pose, roadmap_state
    
    