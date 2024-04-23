#!/usr/bin/env python3
""" 

Module that performs grasping from 6D object poses

Lecture author: Rui Pimentel de Figueiredo

"""
import moveit_commander
from scipy.spatial.transform import Rotation 
import rospy
import sys
import tf2_ros 

from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped

###############################################
# Class with methods for MoveIt interfacing   #     
############################################### 
  
class MoveitHelpers:
    # Reimplement the methods from previous lectures
    def __init__(self) -> None:
        return

    def arm_move_cartesian_pose_abs(self,pose_target, max_vel=0.1, max_acc=0.1):
        return

    def move_arm_to_pose(self,pose_name):
        return
    def move_hand_to_pose(self,pose_name):
        return

class VisionBasedGraspingROS():
    def __init__(self) -> None:

        self.moveit_helpers = MoveitHelpers()

        # start by moving the arm away from the camera field of view
        self.moveit_helpers.move_arm_to_pose("panda_bin")

        # ROS transform buffer
        self.tf_buffer             = tf2_ros.Buffer(rospy.Duration(100.0))

        # ROS transform listener
        self.tf_listener           = tf2_ros.TransformListener(self.tf_buffer)

        # ROS transform broadcaster
        self.tf_broadcaster        = tf2_ros.TransformBroadcaster()

        # Wrist frame approach direction (Z)
        self.pre_grasp_offset      = rospy.get_param("pre_grasp_offset", 0.1)
        self.robot_base_frame_id   = rospy.get_param("robot_base_frame_id","franka_arm_link0")

        #  ROS Subscribers
        self.object_poses_sub      = rospy.Subscriber('object_poses_in', PoseArray,       self.objects_poses_callback)

    # transform a geometry_msgs/PoseStamped to target frame
    def transform_pose(self, input_pose, target_frame):
        try:
            rospy.loginfo("transform pose from "+ input_pose.header.frame_id+" to " + str(target_frame))
            transformed_pose=self.tf_buffer.transform(input_pose, target_frame, rospy.Duration(1))

            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform pose: %s", str(e))
            return None

    # Utilize moveit interface from previous lectures
    def pick_and_place(self, grasp_pose):

        # 1. Convert pose to arm frame

        # 2. Move the arm to pre-grasping pose

        # 3. Mode the arm to target pose

        # 4. Close the hand to grip the box

        # 5. Lift the box

        # 6. Place the box in the panda_bin location

        return
    #####################
    # Callback methods  #
    #####################

    def objects_poses_callback(self, poses_msg):
        # Iterate over poses array message and attempt pick and place
        for pose in poses_msg.poses:

            # Do any necessary change or transformation to the pose
            pose_changed=pose

            # Attempt pick and place
            self.pick_and_place(pose_changed)
        return
    

def main(args):
    rospy.init_node('grasp_object', anonymous=True)
    vision_based_grasping = VisionBasedGraspingROS()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
