#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def spawn_box_in_moveit():
    """
    Spawns a box as a collision object in the MoveIt planning scene.
    """
    rospy.init_node('spawn_box_in_moveit', anonymous=True)

    # Initialize moveit_commander and a rospy node
    moveit_commander.roscpp_initialize(sys.argv)

    # Connect to the planning scene interface
    scene = PlanningSceneInterface()
    rospy.sleep(1)  # Sleep to allow time for the above connections to be made

    # Define the pose of the box (relative to the world frame)
    box_pose = PoseStamped()
    box_pose.header.frame_id = "world"  # The name of the frame in which the box is defined
    box_pose.pose.position.x = 0.0  # Position of the box
    box_pose.pose.position.y = -1.0
    box_pose.pose.position.z = 1.05
    box_pose.pose.orientation.w = 1.0  # No rotation

    # Add the box to the planning scene
    box_name = "spawned_box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))  # Specify the dimensions of the box

    rospy.loginfo("Box added to the planning scene.")
    rospy.sleep(5)  # Sleep to ensure the box persists before the script ends

if __name__ == '__main__':
    try:
        spawn_box_in_moveit()
    except rospy.ROSInterruptException:
        pass

