#!/usr/bin/env python

import copy
import numpy as np
import sys

import actionlib
import moveit_commander
import rospy
import tf
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from graspit_commander import GraspitCommander
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


# use raw_input for python2 c.f. https://stackoverflow.com/questions/5868506/backwards-compatible-input-calls-in-python
if hasattr(__builtins__, 'raw_input'):
    input = raw_input


class FetchPickAndPlace(object):

    def __init__(self):
        super(FetchPickAndPlace, self).__init__()
        # Initialize graspit
        self.gc = GraspitCommander()
        self.gc.clearWorld()
        self.gc.loadWorld('fetch_pick_and_place')
        # = self.gc.importRobot('fetch_gripper_modified') +
        #   self.gc.importGraspableBody('fetch_pick_and_place_cylinder') +
        #   self.gc.importObstacle('cafe_table_surface') +
        #   moving them to predefined poses and setting simulator viewpoint

        # Initialize head and gripper
        # https://github.com/ZebraDevs/fetch_gazebo/blob/0.9.2/fetch_gazebo/scripts/prepare_simulated_robot_pick_place.py
        rospy.loginfo("Waiting for head_controller...")
        self.head_client = actionlib.SimpleActionClient(
            "head_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self.head_client.wait_for_server()
        rospy.loginfo("...connected.")
        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient(
            "gripper_controller/gripper_action",
            GripperCommandAction,
        )
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected.")

        # Initialize moveit
        ## http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_w_t_commander = moveit_commander.MoveGroupCommander(
            'arm_with_torso')
        self.scene = moveit_commander.PlanningSceneInterface()
        self.scene.clear()

        # Initialize tf
        self.tfl = tf.TransformListener()

        # Move head to initial pose
        self.move_head([0, 0.6], 1.0)

    def move_joints(self, client, joint_names, joint_pos, time):
        if len(joint_names) != len(joint_pos):
            rospy.logerr(
                'Must be the same length: {} and {}'.format(
                    joint_names, joint_pos)
            )
            return False
        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.points.append(JointTrajectoryPoint())
        traj.points[0].positions = joint_pos
        traj.points[0].velocities = [0.0] * len(joint_pos)
        traj.points[0].accelerations = [0.0] * len(joint_pos)
        traj.points[0].time_from_start = rospy.Duration(time)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        goal.goal_time_tolerance = rospy.Duration(0.0)
        client.send_goal(goal)
        client.wait_for_result()
        return True

    def move_head(self, joint_pos, time=5.0):
        self.move_joints(
            self.head_client,
            ['head_pan_joint', 'head_tilt_joint'],
            joint_pos,
            time,
        )

    def move_gripper(self, position, max_effort=10.0):
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

    def wait_for_scene_update(
        self,
        obj_name,
        obj_is_known=False,
        obj_is_attached=False,
        timeout=float('inf'),
    ):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([obj_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = obj_name in self.scene.get_known_object_names()

            if (obj_is_attached == is_attached) and (obj_is_known == is_known):
              return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def execute_plan_until_success(
        self,
        commander,
        plan,
    ):
        res = commander.execute(plan, wait=True)
        goal_pos = plan.joint_trajectory.points[-1].positions
        while not res:
            rospy.loginfo('Re-plan & execute to reach goal...')
            commander.set_joint_value_target(goal_pos)
            plan_add = commander.plan()
            if not plan_add.joint_trajectory.points:
                rospy.logerr('Replanning went wrong. Will retry...')
                continue
            res = commander.execute(plan_add, wait=True)
            if res:
                rospy.loginfo('Finally execution succeeded')
        return res

    def run(self):
        # Grasp planning
        rospy.loginfo('Start grasp planning...')
        planned_grasps_msg = self.gc.planGrasps(max_steps=100000)
        planned_valid_grasps = [
            x for x in sorted(
                planned_grasps_msg.grasps,
                key=lambda x: x.epsilon_quality,
                reverse=True
            )
            if x.epsilon_quality > 0
            # epsilon_quality <= 0 means force closure is not established
        ]
        if len(planned_valid_grasps) == 0:
            rospy.logerr('No valid grasp was found in grasp planning')
            return

        # Hack for real fetch gripper.
        # Real fetch gripper is a parallel gripper and has only one DOF.
        # This makes grasp planning difficult,
        # so we use a gripper model having two DOFs in grasp planning.
        # Here, we modify planning results so that real gripper can execute them.
        real_robot_grasps = []
        for grasp in planned_valid_grasps:
            # Offset to move gripper so that object is located in its center
            offset_gripper = np.array(
                [0, ((grasp.dofs[0] - grasp.dofs[1]) / 2.0) * 0.01, 0, 1])
            grasp_pose_rot = tf.transformations.quaternion_matrix([
                grasp.pose.orientation.x,
                grasp.pose.orientation.y,
                grasp.pose.orientation.z,
                grasp.pose.orientation.w
            ])
            offset_object = np.dot(grasp_pose_rot, offset_gripper)

            # Apply offset
            real_robot_grasp = copy.deepcopy(grasp)
            real_robot_grasp.pose.position.x += offset_object[0]
            real_robot_grasp.pose.position.y += offset_object[1]
            real_robot_grasp.pose.position.z += offset_object[2]
            real_robot_grasp.dofs = [0.0, 0.0]
            real_robot_grasp.dofs[0] = (grasp.dofs[0] + grasp.dofs[1]) / 2.0
            real_robot_grasp.dofs[1] = real_robot_grasp.dofs[0]
            real_robot_grasps.append(real_robot_grasp)

        # Target object pose from base_link
        # TODO: Acquire this from recognition
        object_position = [0.6, -0.1, 0.785]
        object_orientation = [0, 0, 0, 1]
        object_mat = self.tfl.fromTranslationRotation(
            object_position, object_orientation)
        object_pose = PoseStamped()
        object_pose.header.frame_id = 'base_link'
        object_pose.pose.position.x = object_position[0]
        object_pose.pose.position.y = object_position[1]
        object_pose.pose.position.z = object_position[2]
        object_pose.pose.orientation.x = object_orientation[0]
        object_pose.pose.orientation.y = object_orientation[1]
        object_pose.pose.orientation.z = object_orientation[2]
        object_pose.pose.orientation.w = object_orientation[3]
        object_name = 'object'
        object_collision_size = [0.1, 0.1, 0.24]
        # Bigger than original object to prevent robot from grazing object

        # Avoid table in motion planning
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'base_link'
        table_pose.pose.position.x = 1.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.3475
        table_pose.pose.orientation.x = 0
        table_pose.pose.orientation.y = 0
        table_pose.pose.orientation.z = 0
        table_pose.pose.orientation.w = 1.0
        self.scene.add_box('table', table_pose, size=(0.913, 0.913, 0.695))
        self.wait_for_scene_update('table', obj_is_known=True)

        # Motion planning & execution
        for grasp_idx, grasp in enumerate(real_robot_grasps):
            rospy.loginfo('Try grasp candidate No. %d', grasp_idx)
            # Graspit visualization
            self.gc.autoOpen()
            self.gc.setRobotPose(grasp.pose)

            # Avoid object in motion planning
            self.scene.add_box(
                object_name, object_pose, size=object_collision_size)
            self.wait_for_scene_update(object_name, obj_is_known=True)

            # Convert object-relative grasp pose to base_link
            grasp_pose_mat_object = self.tfl.fromTranslationRotation(
                [grasp.pose.position.x,
                 grasp.pose.position.y,
                 grasp.pose.position.z],
                [grasp.pose.orientation.x,
                 grasp.pose.orientation.y,
                 grasp.pose.orientation.z,
                 grasp.pose.orientation.w]
            )
            grasp_pose_mat_base = np.dot(object_mat, grasp_pose_mat_object)
            grasp_pose_pos_base = tf.transformations.translation_from_matrix(
                grasp_pose_mat_base)
            grasp_pose_ori_base = tf.transformations.quaternion_from_matrix(
                grasp_pose_mat_base)
            grasp_pose_base = Pose()
            grasp_pose_base.position.x = grasp_pose_pos_base[0]
            grasp_pose_base.position.y = grasp_pose_pos_base[1]
            grasp_pose_base.position.z = grasp_pose_pos_base[2]
            grasp_pose_base.orientation.x = grasp_pose_ori_base[0]
            grasp_pose_base.orientation.y = grasp_pose_ori_base[1]
            grasp_pose_base.orientation.z = grasp_pose_ori_base[2]
            grasp_pose_base.orientation.w = grasp_pose_ori_base[3]

            # Prepare pregrasp waypoint
            pregrasp_waypoint_gripper = np.array([
                grasp.approach_direction.vector.x,
                grasp.approach_direction.vector.y,
                grasp.approach_direction.vector.z,
                0
            ])
            pregrasp_waypoint_gripper /= np.linalg.norm(
                pregrasp_waypoint_gripper)
            pregrasp_waypoint_gripper *= -0.1  # waypoint is 0.1m before the grasp pose
            pregrasp_waypoint_gripper[3] = 1
            grasp_pose_rot = copy.deepcopy(grasp_pose_mat_base)
            grasp_pose_rot[0][3] = 0
            grasp_pose_rot[1][3] = 0
            grasp_pose_rot[2][3] = 0
            pregrasp_waypoint_diff = np.dot(
                grasp_pose_rot, pregrasp_waypoint_gripper)
            pregrasp_waypoint_pose = copy.deepcopy(grasp_pose_base)
            pregrasp_waypoint_pose.position.x += pregrasp_waypoint_diff[0]
            pregrasp_waypoint_pose.position.y += pregrasp_waypoint_diff[1]
            pregrasp_waypoint_pose.position.z += pregrasp_waypoint_diff[2]

            # Move to pregrasp waypoint
            self.arm_w_t_commander.set_pose_target(pregrasp_waypoint_pose)
            plan_to_pregrasp_waypoint = self.arm_w_t_commander.plan()
            if not plan_to_pregrasp_waypoint.joint_trajectory.points:
                rospy.logerr(
                    'Motion planning to pregrasp waypoint failed. Go to next grasp candidate')
                continue
            self.execute_plan_until_success(
                self.arm_w_t_commander, plan_to_pregrasp_waypoint)
            self.arm_w_t_commander.stop()
            self.arm_w_t_commander.clear_pose_targets()

            # Move to grasp pose
            self.scene.remove_world_object(object_name)
            self.wait_for_scene_update(object_name, obj_is_known=False)
            plan_to_grasp_pose, _ = self.arm_w_t_commander.compute_cartesian_path(
                [grasp_pose_base], 0.01, 0.0)
            if not plan_to_grasp_pose.joint_trajectory.points:
                rospy.logerr(
                    'Motion planning to grasp pose failed. Currently we cannot recover from here')
                return
            self.execute_plan_until_success(
                self.arm_w_t_commander, plan_to_grasp_pose)
            self.arm_w_t_commander.stop()

            # Close gripper
            self.move_gripper((grasp.dofs[0] + grasp.dofs[1]) * 0.01)

            # Pick & place
            from_pick_to_place = []
            from_pick_to_place.append(copy.deepcopy(grasp_pose_base))
            from_pick_to_place[-1].position.z += 0.05
            from_pick_to_place.append(copy.deepcopy(from_pick_to_place[-1]))
            from_pick_to_place[-1].position.y += 0.1
            from_pick_to_place.append(copy.deepcopy(from_pick_to_place[-1]))
            from_pick_to_place[-1].position.z -= 0.05
            plan_to_place, _ = self.arm_w_t_commander.compute_cartesian_path(
                from_pick_to_place, 0.01, 0.0)
            if not plan_to_place.joint_trajectory.points:
                rospy.logerr(
                    'Motion planning to place failed. Currently we cannot recover from here')
                return
            self.execute_plan_until_success(
                self.arm_w_t_commander, plan_to_place)
            self.arm_w_t_commander.stop()

            # Open gripper
            self.move_gripper(0.1)

            # Move gripper away
            postgrasp_pose = self.arm_w_t_commander.get_current_pose().pose
            postgrasp_pose.position.x += pregrasp_waypoint_diff[0]
            postgrasp_pose.position.y += pregrasp_waypoint_diff[1]
            postgrasp_pose.position.z += pregrasp_waypoint_diff[2]
            plan_to_postgrasp, _ = self.arm_w_t_commander.compute_cartesian_path(
                [postgrasp_pose], 0.01, 0.0)
            self.execute_plan_until_success(
                self.arm_w_t_commander, plan_to_postgrasp)
            self.arm_w_t_commander.stop()

            return


if __name__ == '__main__':
    rospy.init_node('fetch_pick_and_place')
    app = FetchPickAndPlace()
    input('Press Enter to start...')
    app.run()
