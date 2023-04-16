#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

rospy.init_node("panda_convert_js_node")
pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

def callback(msg):
    pub_msg = JointState()
    pub_msg.header.stamp = msg.header.stamp
    pub_msg.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']
    pub_msg.position = [msg.position[i] for i, name in enumerate(msg.name) if 'joint' in name]
    pub.publish(pub_msg)

rospy.Subscriber("/joint_mujoco_states", JointState, callback, queue_size=1)
rospy.spin()
