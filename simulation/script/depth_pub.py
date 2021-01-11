#!/usr/bin/env python
#coding=utf-8

import numpy as np
import rospy
#导入自定义的数据类型
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelStates

img_recongition = False
sphere_x,sphere_y,sphere_z = 0,0,0
mav_x,mav_y,mav_z = 0,0,0

def spin():
    rospy.spin()

def depth_cb(msg):
    global img_recongition
    if msg.data[4] == 0:
        img_recongition = False
    else:
        img_recongition = True
    # print(img_recongition)

def states_cb(msg):
    global sphere_x,sphere_y,sphere_z
    global mav_x, mav_y, mav_z
    sphere_x = msg.pose[0].position.x
    sphere_y = msg.pose[0].position.y
    sphere_z = msg.pose[0].position.z
    mav_x = msg.pose[3].position.x
    mav_y = msg.pose[3].position.y
    mav_z = msg.pose[3].position.z

def listener_pose():
    sphere_depth_pub = rospy.Publisher("tracker/depth", PoseStamped, queue_size=10)  # send sphere postion
    rospy.Subscriber('tracker/pos_image', Float32MultiArray, depth_cb)
    rospy.Subscriber('gazebo/model_states', ModelStates, states_cb)
    rospy.init_node('iris_fpv_cam', anonymous=True)

    sphere_depth = PoseStamped()
    print('ok')

    interval_rate = 50
    interval_time = 1.0 / interval_rate
    rate = rospy.Rate(interval_rate)

    while not rospy.is_shutdown():
        dx = (sphere_x - mav_x) ** 2
        dy = (sphere_y - mav_y) ** 2
        dz = (sphere_z - mav_z) ** 2
        dis_uavtosphere = (dx + dy + dz) ** 0.5
        # print("sphere pose: {}\nmav pose: {}".format([sphere_x, sphere_y, sphere_z], [mav_x, mav_y, mav_z]))

        # print("dis_uavtosphere: {}, img_recongition: {}".format(dis_uavtosphere, img_recongition))
        if dis_uavtosphere < 5 and img_recongition == True:
            sphere_depth.pose.position.x = mav_x - sphere_x
            sphere_depth.pose.position.y = mav_y - sphere_y
            sphere_depth.pose.position.z = mav_z - sphere_z
            print("ex: {}, ey: {}, ez: {}".format(dx, dy, dz))

        sphere_depth_pub.publish(sphere_depth)
        # print("sphere_depth: {}".format(sphere_depth))
        rate.sleep()

if __name__ == '__main__':
    listener_pose()