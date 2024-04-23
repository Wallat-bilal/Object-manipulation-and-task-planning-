#!/usr/bin/env python3
""" 

Module that performs 6D object estimation from 2D object detections, and depth information

Lecture author: Rui Pimentel de Figueiredo

"""

import numpy as np

import moveit_commander
from scipy.spatial.transform import Rotation 
import rospy
import sys
from sensor_msgs.msg import PointCloud2, Image, CameraInfo

from geometry_msgs.msg import Pose
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge, CvBridgeError

# use the correct definition
from yolov5_pytorch_ros.msg import BoundingBoxes

import tf2_ros 
from geometry_msgs.msg import PoseArray
import cv2

    
class PoseEstimationROS():
    def __init__(self) -> None:

        # ROS transform buffer
        self.tf_buffer      = tf2_ros.Buffer(rospy.Duration(100.0))

        # ROS transform listener
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer)

        # ROS transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Create camera model
        self.camera_model  =  PinholeCameraModel()

        # Create opencv ros interface
        self.bridge        =  CvBridge()

        # Auxiliary variables
        self.camera_info_available       = False
        self.object_detections_available = False
        self.depth_image_available       = False

         #  ROS Subscribers
        self.camera_info_sub       = rospy.Subscriber('camera_info',                 CameraInfo,           self.camera_info_callback)
        self.object_detections_sub = rospy.Subscriber('object_detections_in',        BoundingBoxes,       self.object_detections_callback)
        self.camera_depth_sub      = rospy.Subscriber('camera_depth_image',          Image,                self.camera_depth_callback)

        # ROS Publishers
        self.object_poses_pub                = rospy.Publisher('object_poses_out', PoseArray)
        self.pointcloud_pub                  = rospy.Publisher('pointcloud_from_depth_pub',    PointCloud2)


    ####################################################
    # Convert the depthmap to a 3D point cloud
    #
    # Input:
    # depth_frame 	 	   : The depth_frame containing the depth map (opencv)
    # camera_intrinsics    : The intrinsic values of the imager in whose coordinate system the depth_frame is computed (image geometry PinholeCameraModel)
    # crop (optional)      : Bounding box crop in depth image plane
    # Return:
    # point_cloud          : 3D point cloud
    #
    #####################################################
    def convert_depth_image_to_pointcloud(self,depth_image, crop=None):
        x_min=0
        y_min=0
        height,width = depth_image.shape

        if crop is not None:
            x_min=crop[0] 
            y_min=crop[1] 
            width=crop[2] 
            height=crop[3]
            depth_image=depth_image[x_min:x_min+width, y_min:y_min+height]

        point_cloud=np.zeros((width,height,3), dtype=np.float32)

        # COMPLETE: convert cropped depth to point cloud. Hint use the self.camera_model and projective geometry


        return point_cloud

   
    ##############################################################
    # Input: 
    #   3D point cloud, point cloud header, and object frame id
    # Output:
    #   object_pose:  geometry_msgs/PoseStamped()                
    ##############################################################
    def compute_pose(self, point_cloud, header):
        try:

            # compute pose from Principal Component Analysis of object point cloud
            object_pose=Pose()
            object_pose.header=header

            return object_pose
        except:
            return
        

    def convert_detection_msg_to_bounding_box(self, bounding_box_msg):
        try:

            bounding_box=(0,0,0,0)
            return bounding_box
        except:
            return None
        
    #####################
    # Callback methods  #
    #####################

    # Depth image callback (sensors_msgs/Image)
    def camera_depth_callback(self,img_depth):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(img_depth, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
        if self.camera_info_available==True:
            self.depth_image_available=True

    def camera_info_callback(self,intrinsics_msg):
        # initialize the shape name and approximate the contour
        self.camera_model.fromCameraInfo(intrinsics_msg)
        self.camera_info_available=True

    def object_detections_callback(self, detections_msg):
        # if camera info and depth image is available iterate over detections and compute 6D poses using depth information
        if self.depth_image_available and self.camera_info_available:
            objects_poses_array_msg=PoseArray()
            objects_poses_array_msg.header=self.depth_image.header
            for bounding_box_msg in detections_msg.bounding_boxes:
                # convert bounding box to (x_min,y_min,width,height) format
                bounding_box=self.convert_detection_msg_to_bounding_box(bounding_box_msg)

                # extract object point cloud from detection bounding box and depth image
                object_point_cloud=self.convert_depth_image_to_pointcloud(bounding_box)
                
                # compute 6D pose using point_cloud information
                pose=self.compute_pose(object_point_cloud, self.depth_image)

                # add pose msg to poses array msg
                objects_poses_array_msg.poses.append(pose)

                break
            # publish poses
            self.object_poses_pub.publish(objects_poses_array_msg)
        return
    
def main(args):
    rospy.init_node('pose_estimation', anonymous=True)
    pose_estimation = PoseEstimationROS()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)