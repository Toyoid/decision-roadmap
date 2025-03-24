from gazebo_msgs.msg import ContactsState
import rospy

collision = False
contact_data = None


def collision_callback(msg):
    global collision
    global contact_data
    if msg.states:
        collision = True
    else:
        collision = False
    contact_data = msg


if __name__ == "__main__":
    rospy.init_node('collision_check_test_node')
    contact_state_sub = rospy.Subscriber("/quadcopter/bumper_states", ContactsState, collision_callback)
    rate = rospy.Rate(50)

    while not contact_data:
        rospy.loginfo("Waiting for contact sensor data...")
        continue

    while not rospy.is_shutdown():
        if collision:
            print("----------------------------")
            print(f"Contact State: {contact_data.states}")
            print("----------------------------")
            print(" ")

        rate.sleep()