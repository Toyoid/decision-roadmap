#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


depth_img = None


def _depth_img_callback(img):
    global depth_img
    depth_img = img


if __name__ == '__main__':
    rospy.init_node('depth_img_test_node')
    depth_img_sub = rospy.Subscriber("/camera/depth/image_raw", Image, _depth_img_callback)
    rate = rospy.Rate(100)

    bridge = CvBridge()

    while not depth_img:
        rospy.loginfo("Waiting for camera data...")
        rospy.sleep(0.5)

    stacked_imgs = np.dstack([np.zeros((480, 640))] * 10)
    while not rospy.is_shutdown():
        # cv_img = bridge.imgmsg_to_cv2(depth_img, desired_encoding='32FC1')
        cv_img = bridge.imgmsg_to_cv2(depth_img, desired_encoding='passthrough')
        # cv_img = np.array(cv_img, dtype=np.int8)
        cv_img = np.array(cv_img)
        # cv_img[np.isnan(cv_img)] = 99
        print("************************************************")
        print(f"Converted Depth Image: \n"
              f"dtype: {cv_img.dtype} \n"
              f"shape: {cv_img.shape} \n"
              f"max: {cv_img.max()}, min: {cv_img.min()} \n"  # max depth: 
              f"data: {cv_img} \n")

        # scaled_img = cv_img / 5.0
        # print("************************************************")
        # print(f"Scaled Stacked Image: \n"
        #       f"dtype: {scaled_img.dtype} \n"
        #       f"shape: {scaled_img.shape} \n"
        #       f"max: {scaled_img.max()}, min: {scaled_img.min()} \n"
        #       f"data: {scaled_img} \n")

        stacked_imgs = np.dstack([stacked_imgs[:, :, -9:], cv_img])
        # print("************************************************")
        # print(f"Stacked Image: \n"
        #       f"dtype: {stacked_imgs.dtype} \n"
        #       f"shape: {stacked_imgs.shape} \n"
        #       f"max: {stacked_imgs.max()}, min: {stacked_imgs.min()} \n"
        #       f"data: {stacked_imgs} \n")

        # scaled_stacked_imgs = stacked_imgs / 5.0
        # print("************************************************")
        # print(f"Scaled Stacked Image: \n"
        #       f"dtype: {scaled_stacked_imgs.dtype} \n"
        #       f"shape: {scaled_stacked_imgs.shape} \n"
        #       f"max: {scaled_stacked_imgs.max()}, min: {scaled_stacked_imgs.min()} \n"
        #       f"data: {scaled_stacked_imgs} \n")
        print("************************************************")
        print(" ")

        rate.sleep()

