#!/usr/bin/env python3

import os
import copy
import sys
import glob
import pprint
import random
import datetime
import argparse
import numpy as np

import tf
import rospy
from rospkg import RosPack
from std_msgs.msg import Float32MultiArray, String
from jsk_recognition_msgs.msg import BoundingBoxArray

import pinocchio
from panda_ik import solve_ik

def main():
    rospy.init_node("panda_grasp_node")
    ctrl_ref_pub = rospy.Publisher("mujoco_ctrl_ref", Float32MultiArray, queue_size=1)
    listener = tf.TransformListener()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            obj_pos, _ = listener.lookupTransform("/panda_link0", "/panda_sim/euclidean_clustering_decomposeroutput00", rospy.Time(0))
            obj_pos = np.array(obj_pos, dtype=np.float32)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue
    print(obj_pos)

    oMdeses  = [
            (pinocchio.SE3(pinocchio.utils.rpyToMatrix(np.deg2rad(180), 0, 0), obj_pos-np.array([0, 0, -0.3])), False, 2.0, 5.0),
            (pinocchio.SE3(pinocchio.utils.rpyToMatrix(np.deg2rad(180), 0, 0), obj_pos-np.array([0, 0, -0.05])), False, 2.0, 5.0),
            (pinocchio.SE3(pinocchio.utils.rpyToMatrix(np.deg2rad(180), 0, 0), obj_pos-np.array([0, 0, -0.05])), True, 1.0, 3.0),
            (pinocchio.SE3(pinocchio.utils.rpyToMatrix(np.deg2rad(180), 0, 0), obj_pos-np.array([0, 0, -0.3])), True, 2.0, 5.0),
            ]

    for i_oMdes, (oMdes, grasp_bool, move_time, sleep_time) in enumerate(oMdeses):
        q, success, i_ik, err = solve_ik(oMdes)
        q = q[:-1]
        q[-1] = 0 if grasp_bool else 255

        print("---------------------")
        if success:
            print("Success: {}".format(i_oMdes))
        else:
            print("Failed: {}".format(i_oMdes))
        print('iteration: %s' % i_ik)
        print('result: %s' % q.flatten().tolist())
        print('final error: %s' % err.T)
        ctrl_ref_msg = Float32MultiArray()
        ctrl_ref_msg.data = np.concatenate([q.T, np.array([move_time])])
        ctrl_ref_pub.publish(ctrl_ref_msg)
        rospy.sleep(rospy.Duration(sleep_time))


if __name__ == '__main__':
    main()

