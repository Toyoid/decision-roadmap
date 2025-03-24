import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from global_planner.srv import GetRoadmap
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from scipy.spatial import KDTree
import os


class RoadmapService:
    def __init__(self):
        self.odom = None
        self.node_coords = None
        self.node_utility_inputs = None
        self.edges_adj_pos = []
        self.edges_adj_idx = []
        self.robot_position = []
        self.current_node_idx = None
        self.k_neighbor_size = 20

        rospy.init_node('roadmap_service_node')
        self.odom_sub = rospy.Subscriber("/CERLAB/quadcopter/odom", Odometry, self.odom_callback)
        self.get_roadmap = rospy.ServiceProxy('/dep/get_roadmap', GetRoadmap)
        self.current_goal_pub = rospy.Publisher("/agent/current_goal", PointStamped, queue_size=5)
        self.robot_odom_init = False

        # plot preparation
        self.fig, (self.ax1, self.ax2) = plt.subplots(nrows=1, ncols=2, figsize=(20, 8))
        self.ax1.set_xlim(-30, 30)
        self.ax1.set_ylim(-30, 30)
        self.ax2.set_xlim(-30, 30)
        self.ax2.set_ylim(-30, 30)
        # make width = height in each subplot
        self.ax1.set_aspect('equal')
        self.ax2.set_aspect('equal')
        self.plot_line_list1 = []
        self.plot_patch_list1 = []
        self.plot_line_list2 = []
        self.plot_patch_list2 = []

        # Init Subscriber
        while not self.robot_odom_init:
            continue
        rospy.loginfo("Finish Odom Subscriber Init...")

        print("[Roadmap]: Roadmap service initialized, now please specify waypoints for the drone to explore the env...")

    def update_roadmap_data(self):
        self.robot_position = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z]

        rospy.wait_for_service('/dep/get_roadmap')
        try:
            roadmap_resp = self.get_roadmap()
        except rospy.ServiceException as e:
            print("Get Roadmap Service Failed: %s" % e)

        # acquire node position, node utility, edges
        node_pos = []
        node_utility = []
        self.edges_adj_pos = []  # (num_nodes, adjacent_node_positions)
        self.edges_adj_idx = []  # (num_nodes, adjacent_node_indexes)
        for marker in roadmap_resp.roadmapMarkers.markers:
            if marker.ns == 'prm_point':
                pos = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
                # pos = [marker.pose.position.x, marker.pose.position.y]
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
                self.edges_adj_pos.append(edges_from_current_node)

        self.node_coords = np.round(node_pos, 4)
        n_nodes = self.node_coords.shape[0]
        node_utility_inputs = np.array(node_utility).reshape(n_nodes, 1)
        self.node_utility_inputs = node_utility_inputs / 4000.

        # compute current node index
        self.current_node_idx = np.argmin(np.linalg.norm(self.node_coords - self.robot_position[:3], axis=1))

        # compute edges
        for adj_nodes in self.edges_adj_pos:
            adj_node_idxs = []
            for adj_node_pos in adj_nodes:
                assert len(pos) == self.node_coords.shape[1], "Wrong dimension on node coordinates"
                # pos = [adj_node_pos.x, adj_node_pos.y, adj_node_pos.z]
                pos = np.round([adj_node_pos.x, adj_node_pos.y, adj_node_pos.z], 4)
                idx = np.argwhere((self.node_coords == pos).all(axis=1)).item()
                adj_node_idxs.append(idx)
            self.edges_adj_idx.append(adj_node_idxs)

        return node_pos, node_utility, self.edges_adj_pos

    def plot_roadmap(self):
        # clear figure
        [line.pop(0).remove() for line in self.plot_line_list1]
        [patch.remove() for patch in self.plot_patch_list1]
        [line.pop(0).remove() for line in self.plot_line_list2]
        [patch.remove() for patch in self.plot_patch_list2]
        self.plot_line_list1 = []
        self.plot_patch_list1 = []
        self.plot_line_list2 = []
        self.plot_patch_list2 = []

        # Figure 1: the global roadmap
        # edges
        for node_idx, adj_nodes_idx in enumerate(self.edges_adj_idx):
            for adj_node_idx in adj_nodes_idx:
                self.plot_line_list1.append(
                    self.ax1.plot([self.node_coords[node_idx, 0], self.node_coords[adj_node_idx, 0]],
                                  [self.node_coords[node_idx, 1], self.node_coords[adj_node_idx, 1]], linewidth=1.5,
                                  color='darkcyan', zorder=0))
        # nodes
        nodes = self.ax1.scatter(self.node_coords[:, 0], self.node_coords[:, 1],
                            c=self.node_utility_inputs[:], cmap='viridis', alpha=1, zorder=1)
        self.plot_patch_list1.append(nodes)

        robot = patches.Circle(xy=(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y), radius=0.2,
                               color='r', alpha=0.8)
        robot.set_zorder(3)
        self.ax1.add_patch(robot)
        self.plot_patch_list1.append(robot)

        # Figure 2: current node and edges
        # pad edge_inputs_ with k-nearest neighbors
        current_node_position = self.node_coords[self.current_node_idx]
        # build kd-tree to search k-nearest neighbors
        kdtree = KDTree(self.node_coords)
        k = min(self.k_neighbor_size, self.node_coords.shape[0])
        _, nearest_indices = kdtree.query(current_node_position, k=k)

        current_node_edges = self.edges_adj_pos[self.current_node_idx]
        current_node_pos = current_node_edges[0]
        # edges
        for adj_node in current_node_edges:
            self.plot_line_list2.append(
                self.ax2.plot([current_node_pos.x, adj_node.x],
                              [current_node_pos.y, adj_node.y], linewidth=1.5, color='darkcyan', zorder=0))
        # nodes
        nodes = self.ax2.scatter(self.node_coords[nearest_indices, 0], self.node_coords[nearest_indices, 1],
                                 c='darkcyan', alpha=1, zorder=1)
        self.plot_patch_list2.append(nodes)
        # # text
        # for node_idx in nearest_indices:
        #     text = self.ax2.text(self.node_coords[node_idx, 0], self.node_coords[node_idx, 1], f'{node_idx}', fontsize=10, ha='right')
        #     self.plot_patch_list2.append(text)
        # current node
        current = patches.Circle(xy=(current_node_pos.x, current_node_pos.y), radius=0.15, color='r', alpha=0.8)
        current.set_zorder(2)
        self.ax2.add_patch(current)
        self.plot_patch_list2.append(current)

        plt.pause(1)

    @staticmethod
    def close():
        print(" ")
        rospy.loginfo("Shutting down all nodes...")
        os.system("rosnode kill -a")

    def odom_callback(self, msg):
        if self.robot_odom_init is False:
            self.robot_odom_init = True
        self.odom = msg