#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def move_to_pose(pose_name):
    """Moves the Panda arm to a named pose."""
    group.set_named_target(pose_name)
    plan = group.go(wait=True)
    group.stop()  # Ensure that there's no residual movement
    group.clear_pose_targets()

def move_relative(x, y, z):
    """Moves the Panda arm relative to its current position."""
    current_pose = group.get_current_pose().pose
    move_pose = geometry_msgs.msg.Pose()
    move_pose.position.x = current_pose.position.x + x
    move_pose.position.y = current_pose.position.y + y
    move_pose.position.z = current_pose.position.z + z
    move_pose.orientation = current_pose.orientation

    group.set_pose_target(move_pose)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('panda_move_sequence', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    # Sequence of moves
    move_to_pose('panda1_ready')
    move_to_pose('panda1_home')
    move_to_pose('panda1_pregrasp')
    move_to_pose('panda1_place')
    move_relative(0, 0, 0.2)  # Move up by 0.2m

