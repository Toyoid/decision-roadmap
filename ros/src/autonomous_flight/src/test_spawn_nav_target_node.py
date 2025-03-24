# -*- coding: utf-8 -*-
# !/usr/bin/env python

import os
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point


# 定义生成模型的函数
def spawn_navigation_target():
    model_name = "navigation_target"
    model_path = "/home/toy/.gazebo/models/navigation_target/model.sdf"
    initial_pose = Pose(position=Point(x=1, y=0, z=5))

    # 从文件加载模型
    with open(model_path, "r") as f:
        model_xml = f.read()

    # 调用Gazebo的SpawnModel服务
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    resp_sdf = spawn_model(model_name, model_xml, "", initial_pose, "world")

    if resp_sdf.success:
        rospy.loginfo("模型 '{}' 生成成功。".format(model_name))
    else:
        rospy.logerr("模型 '{}' 生成失败。".format(model_name))


# 调用生成模型的函数
if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('spawn_navigation_target')
    try:
        spawn_navigation_target()
    except rospy.ROSInterruptException:
        pass