#!/usr/bin/env python

import os
import sys
import argparse
import numpy as np

import rospy
from rospkg import RosPack
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import JointState, Image, CameraInfo

import mujoco
import mujoco_viewer

class MujocoSim():
    def __init__(self, model_name):

        # initialization of ros node
        rospy.init_node("mujoco_sim")
        self.rospack = RosPack()

        # initialization of model, data, viewer of mujoco
        model_path = self.rospack.get_path("mujoco_tutorials") + "/models/" + model_name
        print("# model path: {}".format(model_path))
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        # generate mask for each joint type
        joint_mask_free = np.where(self.model.jnt_type==0) # pick just the free joints (7 DoF)
        joint_mask_ball = np.where(self.model.jnt_type==1) # pick just the ball joints (4 DoF)
        joint_mask_else = np.where(self.model.jnt_type>1) # pick the slide (2) and hinge (3) joints (1 DoF)

        # construct JointState message
        jointstates_msg = JointState()
        joint_names = [self.model.joint(i).name for i in range(self.model.njnt)]
        joint_names_free = list(np.array(joint_names)[joint_mask_free])
        joint_names_ball = list(np.array(joint_names)[joint_mask_ball])
        joint_names_else = list(np.array(joint_names)[joint_mask_else])

        # attach suffixes to joint names of free&ball joints to specify their DoFs
        jointstates_msg.name = []
        for joint_name_free in joint_names_free:
            jointstates_msg.name += ["{}_{}".format(joint_name_free, suffix) for suffix in ["x","y","z","q0","q1","q2","q3"]]
        for joint_name_ball in joint_names_ball:
            jointstates_msg.name += ["{}_{}".format(joint_name_ball, suffix) for suffix in ["q0","q1","q2","q3"]]
        jointstates_msg.name += joint_names_else
        jointstates_msg.effort = np.zeros(len(jointstates_msg.name))
        jointstates_msg.velocity = np.zeros(len(jointstates_msg.name))

        # create masks that save the qpos indices for each joint type.
        qpos_mask_free = (np.arange(7)+self.model.jnt_qposadr[joint_mask_free].reshape(-1,1)).flatten()
        qpos_mask_ball = (np.arange(4)+self.model.jnt_qposadr[joint_mask_ball].reshape(-1,1)).flatten()
        qpos_mask_else = self.model.jnt_qposadr[joint_mask_else]

        mujoco.mj_step(self.model, self.data)
        self.mujoco_command = None
        self.ctrl_ref = None
        self.ctrl_ref_time = None
        self.ctrl_orig = None
        self.ctrl_cur = np.zeros(self.model.nu, dtype=np.float32)
        self.ctrl_cur_time = None

        # set up publishers and subscribers
        rospy.Subscriber("mujoco_command", String, callback=self.mujoco_command_callback, queue_size=1)
        rospy.Subscriber("mujoco_ctrl_ref", Float32MultiArray, callback=self.ctrl_callback, queue_size=1)
        jointstates_pub = rospy.Publisher("joint_mujoco_states", JointState, queue_size=1)

        # construct camera msg
        # camera msg wil be published if your cv_bridge is compiled by Python3.
        self.camera = None
        self.camera_rgb_pub = None
        self.camera_depth_pub = None
        self.camera_info_pub = None
        self.camera_info_msg = None
        self.camera_size = None
        try:
            # sys.path.remove('/opt/ros/'+os.environ["ROS_DISTRO"]+'/lib/python2.7/dist-packages') # please uncomment for ubunt18.04
            import cv2
            # sys.path.append('/opt/ros/'+os.environ["ROS_DISTRO"]+'/lib/python2.7/dist-packages') # please uncomment for ubuntu 18.04
            from cv_bridge import CvBridge
            self.br = CvBridge()
            self.br.cv2_to_imgmsg(np.zeros(24, dtype=np.uint8).reshape(4, 2, 3), encoding='rgb8') # test
            print("This environment can use cameras")
            work_camera = True
        except Exception as e:
            print(e)
            print("This environment cannot use cv_bridge by Python3.")
            work_camera = False
        if work_camera:
            self.camera_rgb_pub = rospy.Publisher("/camera/rgb/image_raw", Image, queue_size=2)
            self.camera_depth_pub = rospy.Publisher("/camera/depth/image_raw", Image, queue_size=2)
            self.camera_info_pub = rospy.Publisher("/camera/rgb/camera_info", CameraInfo, queue_size=2)
            info_msg = CameraInfo()
            w, h = 640, 480
            camera_names = [self.model.cam(i).name for i in range(self.model.ncam)]
            fovy = self.model.cam_fovy[camera_names.index("camera")]
            f = 0.5*h/np.tan(fovy*np.pi/180/2)
            info_msg.header.frame_id = "/camera_frame"
            info_msg.height = h
            info_msg.width = w
            info_msg.distortion_model = "plumb_bob"
            info_msg.D = [0]*5
            info_msg.K = [f, 0, w/2, 0, f, h/2, 0, 0, 1]
            info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            info_msg.P = [f, 0, w/2, 0, 0, f, h/2, 0, 0, 0, 1, 0] # no distorsion model
            self.camera_info_msg = info_msg
            self.camera_size = (w, h)
            hz = 10
            rospy.Timer(rospy.Duration(1.0/hz), self.image_callback)

        # for debug
        # import ipdb
        # ipdb.set_trace()

        # main loop
        while not rospy.is_shutdown():
            # for the reset mujoco_command
            if self.mujoco_command is not None:
                if self.mujoco_command == "reset":
                    mujoco.mj_resetData(self.model, self.data)
                    mujoco.mj_forward(self.model, self.data)
                    self.ctrl_ref = None
                    self.ctrl_ref_time = None
                    self.ctrl_orig = None
                    self.ctrl_cur = np.zeros(self.model.nu, dtype=np.float32)
                    self.ctrl_cur_time = None
                self.mujoco_command = None

            # actuation
            if self.ctrl_ref_time is not None:
                self.ctrl_cur_time += self.model.opt.timestep
                self.ctrl_cur = self.ctrl_orig + (self.ctrl_ref-self.ctrl_orig)/self.ctrl_ref_time*self.ctrl_cur_time
                if self.ctrl_cur_time >= self.ctrl_ref_time:
                    self.ctrl_ref_time = None
                    self.ctrl_ref_time = None
                    self.ctrl_orig = None
                    self.ctrl_cur_time = None
            for i in range(self.model.nu):
                self.data.actuator(i).ctrl[:] = self.ctrl_cur[i:i+1]
            mujoco.mj_step(self.model, self.data, nstep=1)

            # publish rostopic
            current_time = rospy.Time.now()
            jointstates_msg.header.stamp = current_time
            qpos_free = self.data.qpos[qpos_mask_free]
            qpos_ball = self.data.qpos[qpos_mask_ball]
            qpos_else_raw = self.data.qpos[qpos_mask_else]
            # clip values to within range for joints with limited range.
            qpos_else_clipped = qpos_else_raw*(self.model.jnt_limited[joint_mask_else]==0) \
                    + np.clip(qpos_else_raw, self.model.jnt_range[joint_mask_else,0], self.model.jnt_range[joint_mask_else,1])*(self.model.jnt_limited[joint_mask_else]==1)
            qpos_else_clipped = qpos_else_clipped[0]
            jointstates_msg.position = np.concatenate((qpos_free, qpos_ball, qpos_else_clipped))
            # currently, only output velocity for slide & hinge joints
            jointstates_msg.velocity[qpos_mask_else] = self.data.qvel[self.model.jnt_dofadr][joint_mask_else]
            jointstates_pub.publish(jointstates_msg)

            viewer.render()

    def mujoco_command_callback(self, msg):
        self.mujoco_command = msg.data

    def ctrl_callback(self, msg):
        self.ctrl_ref = np.asarray(msg.data[:-1], dtype=np.float32)
        self.ctrl_ref_time = msg.data[-1]
        self.ctrl_cur_time = 0.0
        self.ctrl_orig = np.asarray([self.data.actuator(i).ctrl[0] for i in range(self.model.nu)])
        if abs(self.ctrl_ref_time) < 1e-6:
            self.ctrl_ref = None
            self.ctrl_ref_time = None
            self.ctrl_orig = None
            self.ctrl_cur = np.asarray(msg.data[:-1], dtype=np.float32)
            self.ctrl_cur_time = None

    def image_callback(self, event):
        timestamp = rospy.Time.now()
        width, height = self.camera_size
        if not hasattr(self, "set_camera_screen"):
            self.set_camera_screen = True
            self.camera_viewer = mujoco_viewer.MujocoViewer(self.model, self.data, 'offscreen', width=width, height=height)
        rgb_img, depth_img = self.camera_viewer.read_pixels(camid=0, depth=True)
        extent = self.model.stat.extent
        znear = self.model.vis.map.znear * extent
        zfar = self.model.vis.map.zfar * extent
        depth_img = znear / (1 - depth_img * (1 - znear / zfar))

        rgb_img_msg = self.br.cv2_to_imgmsg(rgb_img, encoding='rgb8')
        rgb_img_msg.header.stamp = timestamp
        rgb_img_msg.header.frame_id = "camera_frame"
        self.camera_rgb_pub.publish(rgb_img_msg)

        depth_img_msg = self.br.cv2_to_imgmsg(depth_img, encoding='32FC1')
        depth_img_msg.header.stamp = timestamp
        depth_img_msg.header.frame_id = "camera_frame"
        self.camera_depth_pub.publish(depth_img_msg)

        self.camera_info_msg.header.stamp = timestamp
        self.camera_info_msg.header.frame_id = "camera_frame"
        self.camera_info_pub.publish(self.camera_info_msg)

if __name__=="__main__":
    parser = argparse.ArgumentParser(
        description="mujoco simulator with ROS")
    parser.add_argument("--name", '-n', type=str, required=True,
                        help='model file name')
    args = parser.parse_args()

    sim = MujocoSim(args.name)

