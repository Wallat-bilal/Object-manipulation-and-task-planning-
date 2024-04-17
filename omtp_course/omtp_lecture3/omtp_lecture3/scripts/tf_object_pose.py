#!/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Delft University of Technology
# TU Delft Robotics Institute.
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
#  * Neither the name of Delft University of Technology nor the names of its
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
# Modified by: Wallat Bilal
##
import rospy
import sys
import copy
import tf2_ros
import tf2_geometry_msgs  # Import needed for tf2 geometry operations
from geometry_msgs.msg import PoseStamped
from logical_camera.msg import LogicalCameraImage
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
import actionlib

# Initialize the node and MoveIt!
roscpp_initialize(sys.argv)
rospy.init_node('transform_object_pose', anonymous=True)
print("Node and MoveIt! have been initialized.")

# Set up MoveIt! Commanders
robot_arm_group = MoveGroupCommander("panda_arm")
robot_hand_group = MoveGroupCommander("panda_hand")

# Set up Action Clients
robot_arm_client = actionlib.SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)
robot_hand_client = actionlib.SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)
robot_arm_client.wait_for_server()
robot_hand_client.wait_for_server()
rospy.loginfo("Action servers are ready.")

# TF Listener Setup
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
rospy.loginfo("TF Listener is operational.")

def transform_pose(input_pose, from_frame, to_frame):
    """Transforms pose from one frame to another."""
    pose_stamped = PoseStamped()  # Ensure this is PoseStamped from geometry_msgs.msg
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = from_frame
    pose_stamped.pose = input_pose

    try:
        # Ensure tf_buffer is properly defined and accessible here
        return tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1.0))  # Added a timeout duration
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
        rospy.logerr(f"Error transforming pose: {e}")
        return None

def execute_trajectory(group, waypoints, eef_step=0.01):
    """Executes Cartesian path for a group."""
    fraction = 0.0
    for attempt in range(3):
        (plan, fraction) = group.compute_cartesian_path(waypoints, eef_step, 0.0)
        print(f"Successfully planned trajectory fraction: {fraction}")
        if fraction == 1.0:
            break
    if fraction == 1.0:
        goal = ExecuteTrajectoryGoal()
        goal.trajectory = plan
        client = robot_arm_client if group == robot_arm_group else robot_hand_client
        client.send_goal_and_wait(goal)
    return fraction

def logical_camera_callback(data):
    """Processes data from the logical camera and operates the robot."""
    box_seen = any(obj.type == 'box' for obj in data.models)
    print("Attempting to reach box")
    if not box_seen:
        rospy.loginfo("No box detected.")
        return

    for gazebo_object in data.models:
        if gazebo_object.type == 'box':
            object_pose_world = transform_pose(gazebo_object.pose, "logical_camera1_link", "world")
            if object_pose_world is None:
                continue

            rospy.loginfo(f"Transformed pose: {object_pose_world.pose}")
            
            # Open the hand before moving
            robot_hand_group.set_named_target("hand_open")
            robot_hand_group.go()

            # Move to the object position above it
            waypoints = [copy.deepcopy(object_pose_world.pose)]
            waypoints[0].position.z += 0.5  # Offset by 0.5 meters in z-axis
            execute_trajectory(robot_arm_group, waypoints)

            # Move down to the object
            waypoints[0].position.z -= 0.38  # Adjust the z position close to the object
            execute_trajectory(robot_arm_group, waypoints)

            # Close the hand to grasp the object
            robot_hand_group.set_named_target("hand_close")
            robot_hand_group.go()

            # Move back to the ready position
            robot_arm_group.set_named_target("robot_ready")
            robot_arm_group.go()

            rospy.signal_shutdown("Task completed.")
            break

if __name__ == '__main__':
    sub = rospy.Subscriber('/omtp/logical_camera1', LogicalCameraImage, logical_camera_callback)
    rospy.spin()

