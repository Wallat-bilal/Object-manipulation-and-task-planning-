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

from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped, Pose
from nav_msgs.msg import Path
from tf2_geometry_msgs import PoseStamped

###############################################
# Class with methods for MoveIt interfacing   #     
############################################### 
  
class MoveitHelpers:
    # Reimplement the methods from previous lectures
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.loginfo('Moveit script started for Franka robot.')

        # Create move group interface for the arm and hand
        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.hand_group = moveit_commander.MoveGroupCommander("panda_hand")
        rospy.loginfo('Move group commanders for arm and hand initialized.')

    def arm_move_cartesian_pose_abs(self,pose_target, max_vel=0.1, max_acc=0.1):
        rospy.loginfo('- Starting movement to cartesian pose.')
        curr_pose = self.arm_group.get_current_pose()
        print(f"Current pose: {curr_pose}")
        print(f"Target pose: {pose_target}")
        # target = Pose()
        # target.position = pose_target.pose.position
        # target.orientation = curr_pose.orientation
        
        self.arm_group.set_pose_target(pose_target)
        success = self.arm_group.go(wait=True)
        if success:
            self.arm_group.stop()
            rospy.loginfo('- Move successful')
        else:
            self.arm_group.stop()
            rospy.logwarn(' - Movement failed')


    def move_arm_to_pose(self,pose_name):
        rospy.loginfo(f'- Moving arm to {pose_name}.')	
        self.arm_group.set_named_target(pose_name)
        plan_success, plan, planning_time, error_code = self.arm_group.plan()
        if plan_success:
            rospy.loginfo(f'Planning successful. Execution time: {planning_time} seconds.')
            self.arm_group.execute(plan, wait=True)
            rospy.loginfo(f'Arm moved to {pose_name} pose.')
        else:
            rospy.logwarn(f'Arm planning to {pose_name} failed with error code: {error_code}.')
        return
    def move_hand_to_pose(self,pose_name):
        rospy.loginfo(f'- Moving hand to {pose_name}.')
        self.hand_group.set_named_target(pose_name)
        plan_success, plan, planning_time, error_code = self.hand_group.plan()
        if plan_success:
            rospy.loginfo(f'Planning successful. Execution time: {planning_time} seconds.')
            self.hand_group.execute(plan, wait=True)
            rospy.loginfo(f'Hand moved to {pose_name} pose.')
        else:
            rospy.logwarn(f'Hand planning to {pose_name} failed with error code: {error_code}.')

class VisionBasedGraspingROS():
    def __init__(self) -> None:

        self.moveit_helpers = MoveitHelpers()

        # start by moving the arm away from the camera field of view
        #self.moveit_helpers.move_arm_to_pose("panda_bin")

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
        self.object_poses_sub      = rospy.Subscriber('object_poses_in', Path,       self.objects_poses_callback)

    # transform a geometry_msgs/PoseStamped to target frame
    def transform_pose(self, input_pose, target_frame):
        try:
            # rospy.loginfo("transform pose from "+ input_pose.header.frame_id+" to " + str(target_frame))
            transformed_pose=self.tf_buffer.transform(input_pose, target_frame, rospy.Duration(1))
            print(transformed_pose)

            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform pose: %s", str(e))
            return None

    # Utilize moveit interface from previous lectures
    def pick_and_place(self, grasp_pose):

        # 1. Convert pose to arm frame
        pose_target = self.transform_pose(grasp_pose, self.robot_base_frame_id)

        # 2. Move the arm to pre-grasping pose
        pre_grasp_pose = Pose()
        crr_pose = self.moveit_helpers.arm_group.get_current_pose()
        pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z = 0, 0, 0.6
        pre_grasp_pose.orientation = crr_pose.pose.orientation
        self.moveit_helpers.arm_move_cartesian_pose_abs(pre_grasp_pose)
        
        # pre_grasp_pose = pose_target
        # pre_grasp_pose.pose.position.z = grasp_pose.pose.position.z + self.pre_grasp_offset
        
        # self.moveit_helpers.move_hand_to_pose("panda_hand_open")

        # # 3. Mode the arm to target pose
        # self.moveit_helpers.arm_move_cartesian_pose_abs(grasp_pose)

        # # 4. Close the hand to grip the box
        # self.moveit_helpers.move_hand_to_pose("panda_hand_close")

        # # 5. Lift the box
        # self.moveit_helpers.arm_move_cartesian_pose_abs(pre_grasp_pose)

        # # 6. Place the box in the panda_bin location
        # self.moveit_helpers.move_arm_to_pose("panda_bin")
        # self.moveit_helpers.move_hand_to_pose("panda_hand_open")

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
