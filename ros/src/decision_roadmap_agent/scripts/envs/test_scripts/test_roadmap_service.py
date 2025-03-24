import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from global_planner.srv import GetRoadmap
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random


class PRMNode:
    def __init__(self, x=0, y=0, z=0, utility=0):
        self.x = x
        self.y = y
        self.z = z
        self.utility = utility


class PRMEdge:
    def __init__(self, p1, p2):
        # p1, p2: class p (p.x, p.y, p.z)
        self.point1 = p1
        self.point2 = p2


# prm = []
prm = {}
odom_ = None


def odom_callback(msg):
    global odom_
    odom_ = msg


if __name__ == "__main__":
    rospy.init_node('test_roadmap_service_node')
    odom_sub = rospy.Subscriber("/CERLAB/quadcopter/odom", Odometry, odom_callback)
    get_roadmap = rospy.ServiceProxy('/dep/get_roadmap', GetRoadmap)
    # current_goal_pub = rospy.Publisher("/falco_planner/way_point", PointStamped, queue_size=5)
    current_goal_pub = rospy.Publisher("/agent/current_goal", PointStamped, queue_size=5)

    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    plot_line_list = []
    plot_patch_list = []

    goal = PointStamped()
    goal.header.frame_id = "map"

    while not rospy.is_shutdown():
        # get PRM data
        rospy.wait_for_service('/dep/get_roadmap')
        try:
            roadmap_resp = get_roadmap()
        except rospy.ServiceException as e:
            print("Get Roadmap Service Failed: %s" % e)

        # PRM data processing
        nodes = []
        edges = []
        for marker in roadmap_resp.roadmapMarkers.markers:
            if marker.ns == 'prm_point':
                node = PRMNode(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
                node_id = marker.id
                nodes.append(node)
            elif marker.ns == 'num_voxel_text':
                assert len(nodes) > 0, "PRM node list is empty"
                # utility_match = round(marker.pose.position.x, 4) == round(nodes[-1].x, 4) \
                #                 and round(marker.pose.position.y, 4) == round(nodes[-1].y, 4) \
                #                 and round(marker.pose.position.z - 0.1, 4) == round(nodes[-1].z, 4)
                utility_match = marker.id == node_id
                assert utility_match, "Utility does not match with PRM node"
                nodes[-1].utility = int(marker.text)
            elif marker.ns == 'edge':
                p1 = marker.points[0]
                for i, p in enumerate(marker.points):
                    if (i % 2) == 1:
                        p2 = p
                        edge = PRMEdge(p1, p2)
                        edges.append(edge)

        prm = {'nodes': nodes, 'edges': edges}

        # find the node with highest utility
        # best_node = max(prm["nodes"], key=lambda node: node.utility)
        best_node = random.choice(prm["nodes"])

        goal.header.stamp = rospy.Time.now()
        goal.point.x = best_node.x
        goal.point.y = best_node.y
        goal.point.z = best_node.z
        
        current_goal_pub.publish(goal)

        # plot
        for e in prm['edges']:
            plot_line_list.append(
                ax.plot([e.point1.x, e.point2.x], [e.point1.y, e.point2.y], linewidth=1.5, color='darkcyan', zorder=0))

        scatter = ax.scatter([n.x for n in prm['nodes']], [n.y for n in prm['nodes']],
                             c=[n.utility / 4000 for n in prm['nodes']], cmap='viridis', alpha=1, zorder=1)
        plot_patch_list.append(scatter)

        robot = patches.Circle(xy=(odom_.pose.pose.position.x, odom_.pose.pose.position.y), radius=0.2, color='r',
                               alpha=0.8)
        robot.set_zorder(2)
        ax.add_patch(robot)
        plot_patch_list.append(robot)

        # rate.sleep()
        plt.pause(5)

        # clear figure
        [line.pop(0).remove() for line in plot_line_list]
        [patch.remove() for patch in plot_patch_list]
        plot_line_list = []
        plot_patch_list = []
