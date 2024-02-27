#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports

import os
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from scipy.spatial.transform import Rotation as R
import numpy as np
from geometry_msgs.msg import PoseStamped
import cv2
from PIL import Image
import time
import subprocess

import requests
import json
import base64

from gsam import *

endpoint = "http://localhost:8080"
api_path_gel = "/v1/gel"
api_path_rs = "/v1/marker"
api_path_line_gel = "/v1/line_gel"
url_gel = endpoint + api_path_gel
url_rs = endpoint + api_path_rs
url_line_gel = endpoint + api_path_line_gel

method = "GET"
session = requests.Session()

def get_gel_from_api():
    for _ in range(10):
        payload = json.loads(session.get(url_gel).content)
    img = cv2.imdecode(np.frombuffer(base64.b64decode(payload['blob']), dtype=np.uint8), cv2.IMREAD_UNCHANGED)

    return img

def get_line_gel_from_api():
    for _ in range(10):
        payload = json.loads(session.get(url_line_gel).content)
    img = cv2.imdecode(np.frombuffer(base64.b64decode(payload['blob']), dtype=np.uint8), cv2.IMREAD_UNCHANGED)

    return img

def get_marker_from_api():
    payload = json.loads(session.get(url_rs).content)
    marker_pos = payload['blob'][0]
    marker_quat = payload['blob'][1]

    return marker_pos, marker_quat

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


# For SCP process
def run_cmd(cmd_str='') -> subprocess.Popen:
    pipe = subprocess.Popen(cmd_str, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return pipe
    # run(cmd_str, shell=True)

# def run_cmd(cmd_string):
#     import subprocess
    
#     print('运行cmd指令：{}'.format(cmd_string))
#     return subprocess.Popen(cmd_string, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, encoding='utf-8').communicate()[0]

# Franka cam to franka base
def cam_to_tcp():
    # translation=
    x=0.07052626378868507
    y=-0.04014062369543007
    z=-0.10149507138361483
    cma_to_tcp_tran = np.array([x,y,z])
    # rotation=
    x=0.006439728481752117
    y=-0.01198674572725078
    z=0.7162261754005183
    w=0.6977355612946051

    cma_to_tcp_rota = np.array([x,y,z,w])

    cma_to_tcp_R = R.from_quat(cma_to_tcp_rota)
    cma_to_tcp_mat = cma_to_tcp_R.as_matrix()

    cam_to_tcp = np.zeros((4, 4))
    cam_to_tcp[:3, :3] = cma_to_tcp_mat
    cam_to_tcp[:3, 3] = cma_to_tcp_tran
    cam_to_tcp[3, 3] = 1

    return cam_to_tcp
def tcp_to_base(tran,rota):
    tran = np.array(tran)
    rota = np.array(rota)
    tcp_to_base_R = R.from_quat(rota)
    tcp_to_base_mat = tcp_to_base_R.as_matrix()

    tcp_to_base = np.zeros((4, 4))
    tcp_to_base[:3, :3] = tcp_to_base_mat
    tcp_to_base[:3, 3] = tran
    tcp_to_base[3, 3] = 1

    return tcp_to_base


def cam_to_base(tutorial, market_to_cam):
    # print("market_to_cam tr4ansfrom")

    cam_to_tcp_pos = cam_to_tcp()

    cur_pose, cur_rot = tutorial.get_pose()
    tcp_to_base_pos = tcp_to_base(cur_pose, cur_rot)

    market_to_cam_1 = np.append(market_to_cam, 1)

    return (tcp_to_base_pos @ cam_to_tcp_pos @ market_to_cam_1)[:-1]


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        
        group_name = "panda_manipulator"
        gripper_group_name = "panda_hand"
        
        # group_name = "arm"
        # gripper_group_name = "gripper"
        
        move_group = moveit_commander.MoveGroupCommander(group_name)
        gripper_move_group = moveit_commander.MoveGroupCommander(gripper_group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.group_name = group_name
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.gripper_move_group = gripper_move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        
        self.origin_joint_state = self.move_group.get_current_joint_values()
        self.origin_pose_state = self.move_group.get_current_pose().pose
        
        self.ready_state = [0, -45, 0, -135, 0, 90, 45]
        # self.move_group.set_max_velocity_scaling_factor(0.05)
        # self.move_group.set_max_acceleration_scaling_factor(0.05)

    def get_pose(self):
        pose_end = self.move_group.get_current_pose().pose
        current_pose = [pose_end.position.x, pose_end.position.y, pose_end.position.z]
        current_rot = [pose_end.orientation.x, pose_end.orientation.y, pose_end.orientation.z, pose_end.orientation.w]
        print("Current pose: ", current_pose)
        print("Current rot: ", current_rot)
        return current_pose, current_rot
    
    def go_to_ready_state(self):
        self.go_to_joint_state(self.ready_state, True)
        
    def go_to_origin_joint_state(self):
        move_group = self.move_group
        move_group.go(self.origin_joint_state, wait=True)
        
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        
        origin_quat = [self.origin_pose_state.orientation.x, self.origin_pose_state.orientation.y,
                       self.origin_pose_state.orientation.z, self.origin_pose_state.orientation.w]
        print("Origin quat: ", origin_quat)
        r = R.from_quat(origin_quat)
        mat = r.as_matrix()
        print("Origin matrix: ", mat)
        
        
        return all_close(self.origin_joint_state, current_joints, 0.01)
        
    def close_gripper(self):
        gripper_move_group = self.gripper_move_group
        
        joint_goal = gripper_move_group.get_current_joint_values()
        print("\nCurrent joints values for gripper: ", joint_goal)
        
        if self.group_name.startswith("panda"):
            joint_goal[0] = 0.00001
            joint_goal[1] = 0.00001
        else:
            joint_goal[0] = 1
            joint_goal[1] = 1
        
        gripper_move_group.go(joint_goal, wait=True)

        gripper_move_group.stop()

        current_joints = gripper_move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def open_gripper(self):
        gripper_move_group = self.gripper_move_group
        
        joint_goal = gripper_move_group.get_current_joint_values()
        print("\nCurrent joints values for gripper: ", joint_goal)
        
        if self.group_name.startswith("panda"):
            joint_goal[0] = 0.035
            joint_goal[1] = 0.035
            
        else:
            joint_goal[0] = 11 / 360 * tau
            joint_goal[1] = 11 / 360 * tau
        
        gripper_move_group.go(joint_goal, wait=True)

        gripper_move_group.stop()

        current_joints = gripper_move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_joint_state_relative(self, joint_states, degrees=False):
        move_group = self.move_group
        
        joint_goal = move_group.get_current_joint_values()
        # print("\nOriginal joints values for arm: ", joint_goal)
        for i_joint in range(len(joint_goal)):
            if not degrees:
                joint_goal[i_joint] += joint_states[i_joint]
            else:
                joint_goal[i_joint] += joint_states[i_joint] / 360 * tau
            
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        # print("\nCurrent joints values for arm: ", current_joints)
        
        return all_close(joint_goal, current_joints, 0.01)
        
    def go_to_joint_state(self, joint_states, degrees=False):
        move_group = self.move_group
        
        joint_goal = move_group.get_current_joint_values()
        # print("\nCurrent joints values for arm: ", joint_goal)
        for i_joint in range(len(joint_goal)):
            if not degrees:
                joint_goal[i_joint] = joint_states[i_joint]
            else:
                joint_goal[i_joint] = joint_states[i_joint] / 360 * tau
            
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal_relative(self, pose_goal_r):
        move_group = self.move_group
        pose_goal = move_group.get_current_pose().pose
        # print("Original pose: ", pose_goal)
        
        pose_goal.position.x += pose_goal_r[0]
        pose_goal.position.y += pose_goal_r[1]
        pose_goal.position.z += pose_goal_r[2]
        
        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        # print("Current pose: ", current_pose)
        return all_close(pose_goal, current_pose, 0.01)

        
    def go_to_pose_goal(self, pose_goal_r):
        move_group = self.move_group
        pose_goal = move_group.get_current_pose().pose
        # print("Original pose: ", pose_goal)
        
        pose_goal.position.x = pose_goal_r[0]
        pose_goal.position.y = pose_goal_r[1]
        pose_goal.position.z = pose_goal_r[2]
        
        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        # print("Current pose: ", current_pose)
        return all_close(pose_goal, current_pose, 0.01)
    
    def go_to_rot_relative(self, rot_r):

        move_group = self.move_group
        pose_goal = move_group.get_current_pose().pose
        # print("Original pose: ", pose_goal)

        mat_origin = R.from_quat([pose_goal.orientation.x, pose_goal.orientation.y, 
                                  pose_goal.orientation.z, pose_goal.orientation.w]).as_matrix()
        mat_rot = R.from_euler('xyz', rot_r, degrees=True).as_matrix()
        
        quat_now = R.from_matrix(mat_rot @ mat_origin).as_quat()

        pose_goal.orientation.x = quat_now[0]
        pose_goal.orientation.y = quat_now[1]
        pose_goal.orientation.z = quat_now[2]
        pose_goal.orientation.w = quat_now[3]
        
        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        # print("Current pose: ", current_pose)
        return all_close(pose_goal, current_pose, 0.01)
    
    def plan_cartesian_path_relative(self, list_wpose):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        
        for wpose_relative in list_wpose:
            wpose.position.x += wpose_relative[0]
            wpose.position.y += wpose_relative[1]
            wpose.position.z += wpose_relative[2]
            waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        self.display_trajectory(plan)
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )