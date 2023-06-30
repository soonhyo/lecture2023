#!/usr/bin/env python

import os
import sys
import numpy as np

import rospy
from rospkg import RosPack
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import JointState, Image, CameraInfo

import mujoco
import mujoco_viewer

class MujocoSim():
    def __init__(self):

        # initialization of ros node
        rospy.init_node("mujoco_sim")
        self.rospack = RosPack()

        # initialization of model, data, viewer of mujoco
        model_path = self.rospack.get_path("mujoco_tutorials") + "/models/cloth_robot.xml"
        print("# model path: {}".format(model_path))
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        # construct JointState message
        jointstates_msg = JointState()
        jointstates_msg.name = [self.model.joint(i).name for i in range(self.model.njnt)]
        jointstates_msg.effort = np.zeros(len(jointstates_msg.name))

        self.mujoco_command = None
        self.ctrl_ref = np.zeros(self.model.nu, dtype=np.float32)

        # set up publishers and subscribers
        rospy.Subscriber("mujoco_command", String, callback=self.mujoco_command_callback, queue_size=1)
        rospy.Subscriber("mujoco_ctrl_ref", Float32MultiArray, callback=self.ctrl_callback, queue_size=1)
        jointstates_pub = rospy.Publisher("joint_mujoco_states", JointState, queue_size=1)

        # construct camera msg
        # camera msg wil be published if your cv_bridge is compiled by Python3.
        self.camera = None
        self.camera_pub = None
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
            self.camera_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=2)
            self.camera_info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=2)
            info_msg = CameraInfo()
            w, h = 256, 192
            camera_names = [self.model.cam(i).name for i in range(self.model.ncam)]
            fovy = self.model.cam_fovy[camera_names.index("camera")]
            f = 0.5*h/np.tan(fovy*np.pi/180/2)
            info_msg.header.frame_id = "/camera_optical_frame"
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
                    self.ctrl_ref = np.zeros(self.model.nu, dtype=np.float32)
                    mujoco.mj_forward(self.model, self.data)
                self.mujoco_command = None

            # actuation
            for i in range(self.model.nu):
                self.data.actuator(i).ctrl[:] = self.ctrl_ref[i:i+1] # [rad]
            mujoco.mj_step(self.model, self.data, nstep=1)
            import ipdb; ipdb.set_trace()
            # publish rostopic
            current_time = rospy.Time.now()
            jointstates_msg.header.stamp = current_time
            jointstates_msg.position = self.data.qpos
            jointstates_msg.velocity = self.data.qvel
            jointstates_pub.publish(jointstates_msg)

            viewer.render()

    def mujoco_command_callback(self, msg):
        self.mujoco_command = msg.data

    def ctrl_callback(self, msg):
        self.ctrl_ref = np.asarray(msg.data, dtype=np.float32)

    def image_callback(self, event):
        timestamp = rospy.Time.now()
        width, height = self.camera_size
        if not hasattr(self, "set_camera_screen"):
            self.set_camera_screen = True
            self.camera_viewer = mujoco_viewer.MujocoViewer(self.model, self.data, 'offscreen', width=width, height=height)
        img = np.asarray(self.camera_viewer.read_pixels(camid=0))
        img_msg = self.br.cv2_to_imgmsg(img, encoding='rgb8')
        img_msg.header.stamp = timestamp
        self.camera_pub.publish(img_msg)
        self.camera_info_msg.header.stamp = timestamp
        self.camera_info_pub.publish(self.camera_info_msg)

if __name__=="__main__":
    sim = MujocoSim()

