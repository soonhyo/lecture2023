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
import rospy
from rospkg import RosPack
from std_msgs.msg import Float32MultiArray, String
import pinocchio


rospack = RosPack()
model_path = rospack.get_path("mujoco_tutorials") + "/models/panda_robot.urdf"
model = pinocchio.buildModelFromUrdf(model_path)
geom_model = pinocchio.buildGeomFromUrdf(model, model_path, pinocchio.GeometryType.COLLISION)
[geom_model.removeGeometryObject("panda_leftfinger_" + str(i)) for i in range(4)]
[geom_model.removeGeometryObject("panda_rightfinger_" + str(i)) for i in range(4)]
geom_model.addAllCollisionPairs()
for cp in geom_model.collisionPairs: # adjacent collision is removed
    if cp.first+1 == cp.second:
        geom_model.removeCollisionPair(cp)
data = model.createData()
geom_data = pinocchio.GeometryData(geom_model)

def solve_ik(oMdes, target_frame="panda_hand", debug=True, eps=1e-2, IT_MAX=1000, DT=1e-1, damp=1e-12, check_rot="True", check_jol=True):
    fext = pinocchio.StdVec_Force()
    fext.extend([pinocchio.Force.Zero() for j in model.joints])

    q = None
    error = None
    success = False
    FRAME_ID = model.getFrameId(target_frame);
    q = pinocchio.neutral(model)
    i=0

    # variable for joint limit avoidance
    pre_dHdq = np.zeros(model.nv)

    while True:
        pinocchio.forwardKinematics(model, data, q)
        pinocchio.updateFramePlacements(model, data);
        pinocchio.computeCollisions(model, data, geom_model, geom_data, q, False) # True (quit if one collision is detected)
        # pinocchio.computeCollision(geom_model, geom_data, 0) # single collision detection
        is_collide = False
        for k in range(len(geom_model.collisionPairs)):
            cr = geom_data.collisionResults[k]
            cp = geom_model.collisionPairs[k]
            is_collide |= cr.isCollision()
        dMf = oMdes.actInv(data.oMf[FRAME_ID])
        err = pinocchio.log(dMf).vector
        if check_rot == "False":
            err = err[[0, 1, 2]]
        elif check_rot == "x":
            err = err[[0, 1, 2, 4, 5]]
        elif check_rot == "y":
            err = err[[0, 1, 2, 3, 5]]
        elif check_rot == "z":
            err = err[[0, 1, 2, 3, 4]]
        else:
            err = err[[0, 1, 2, 3, 4, 5]]
        error = np.linalg.norm(err)
        if error < eps and (not is_collide):
            success = True
            break
        if i >= IT_MAX:
            success = False
            break
        W_inv = np.eye(model.nv)
        # calc weight matrix for joint limit avoidance
        jl_margin = 30.0 / 180.0 * 3.1415 #joint limit margin
        if check_jol:
            upper_limit = model.upperPositionLimit
            lower_limit = model.lowerPositionLimit
            dHdq = np.zeros(model.nv)
            for j in range(model.nv):
                ul = upper_limit[j]
                ll = lower_limit[j]
                numerator = (ul - ll)**2 * (2.0*q[j] - ul - ll)
                denominator = 4.0 * (ul - q[j])**2 * (q[j] - ll)**2
                min_denominator = 4.0 * (ul - ll - jl_margin)**2 * jl_margin**2

                if (denominator < min_denominator) or (q[j] >= ul) or (q[j] <= ll):
                    denominator = min_denominator
                dHdq[j] = numerator / denominator
            dHdq = np.abs(dHdq)
            delta_dHdq = dHdq - pre_dHdq
            pre_dHdq = dHdq

            for j in range(model.nv):
                if delta_dHdq[j] >= 0:
                    W_inv[j, j] = 1.0 / (1.0 + dHdq[j])

        pinocchio.computeJointJacobians(model, data, q)
        J = pinocchio.getFrameJacobian(model, data, FRAME_ID, pinocchio.ReferenceFrame.LOCAL)
        if check_rot == "False":
            J = J[[0, 1, 2]]
        elif check_rot == "x":
            J = J[[0, 1, 2, 4, 5]]
        elif check_rot == "y":
            J = J[[0, 1, 2, 3, 5]]
        elif check_rot == "z":
            J = J[[0, 1, 2, 3, 4]]
        else:
            J = J[[0, 1, 2, 3, 4, 5]]
        max_diff = 1.0
        if np.linalg.norm(err) > max_diff:
            err = max_diff / np.linalg.norm(err) * err
        v = - W_inv.dot(J.T.dot(np.linalg.solve(J.dot(W_inv.dot(J.T)) + damp * np.eye(len(J)), err)))

        q = pinocchio.integrate(model, q, v*DT)
        # if not i % 10:
        #     print('%d: error = %s' % (i, err.T))
        #     print(np.linalg.norm(err))
        if check_jol:
            # keep q in joint limit range
            q = np.clip(q, model.lowerPositionLimit + np.ones(model.nv) * jl_margin, model.upperPositionLimit - np.ones(model.nv) * jl_margin)

        i += 1
    return q, success, i, err

def main():
    oMdeses  = [
            (pinocchio.SE3(pinocchio.utils.rpyToMatrix(np.deg2rad(180), 0, 0), np.array([0.50, -0.2, 0.3])), 1.0, 2.0),
            (pinocchio.SE3(pinocchio.utils.rpyToMatrix(np.deg2rad(180), 0, 0), np.array([0.50, 0.2, 0.3])), 1.0, 2.0),
            (pinocchio.SE3(pinocchio.utils.rpyToMatrix(np.deg2rad(180), 0, 0), np.array([0.70, 0.2, 0.3])), 1.0, 2.0),
            (pinocchio.SE3(pinocchio.utils.rpyToMatrix(np.deg2rad(180), 0, 0), np.array([0.70, -0.2, 0.3])), 1.0, 2.0),
            ]

    rospy.init_node("panda_ik_node")
    ctrl_ref_pub = rospy.Publisher("mujoco_ctrl_ref", Float32MultiArray, queue_size=1)
    rospy.sleep(rospy.Duration(0.5))

    for i_oMdes, (oMdes, move_time, sleep_time) in enumerate(oMdeses):
        q, success, i_ik, err = solve_ik(oMdes)
        q = q[:-1]
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
