#!/usr/bin/env python3
""" 

Module that performs 6D object estimation from 2D object detections, and depth information

Lecture author: Rui Pimentel de Figueiredo

"""

import numpy as np
from sklearn.decomposition import PCA

import moveit_commander
from scipy.spatial.transform import Rotation 
import rospy
import sys
from sensor_msgs.msg import PointCloud2, Image, CameraInfo

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import Path
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
        self.object_poses_pub                = rospy.Publisher('object_poses_out', Path)
        self.pointcloud_pub                  = rospy.Publisher('pointcloud_from_depth_pub',   PointCloud2)


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
        # Initialize size variables
        x_max = 0
        y_max= 0

        if crop is not None:
            # The output from the bounding box is used as cropping
            x_min=crop[0] 
            y_min=crop[1] 
            x_max=crop[2] 
            y_max=crop[3]
            
            # Copy the depth image, thus not changing the original, with the cropping ranges given
            depth_image_copy=depth_image[x_min:x_max, y_min:y_max]

        else:
            # Assume the entire image is the detected object (???)
            x_min=0
            y_min=0
            y_max, x_max = depth_image.shape

        # Initialization of the point cloud, with sizes given from the bounding box    
        point_cloud=np.zeros((x_max,y_max,3), dtype=np.float32)
        

        # COMPLETE: convert cropped depth to point cloud. Hint use the self.camera_model and projective geometry
        # Obtain camera intrinsics from camera model
        fx = self.camera_model.fx() # Focal length - X
        fy = self.camera_model.fy() # Focal length - Y
        cx = self.camera_model.cx() # Center pixel - X
        cy = self.camera_model.cy() # Center pixel - Y

        # For every pixel
        for u in range(x_min, x_max, 1):
            for v in range(y_min, y_max, 1):
                
                # Obtain the depth
                z = depth_image[u, v] / 1000.0  # Convert depth from millimeters to meters

                if z == 0:  # Invalid depth
                    point_cloud[u, v] = np.nan
                else:
                    #
                    x = (u - cx) * (z / fx)
                    y = (v - cy) * (z / fy)
                    point_cloud[u, v, :] = [x, y, z]
                    
        return point_cloud

   
    ##############################################################
    # Input: 
    #   3D point cloud, point cloud header, and object frame id
    # Output:
    #   object_pose:  geometry_msgs/PoseStamped()                
    ##############################################################
    def compute_pose(self, point_cloud, header):
        # Define cloud indices as a vector of points
        cloud_indices = point_cloud.reshape(-1, 3)
        
        # Instantiate PCA object
        pca = PCA(n_components=3)
        
        # Attempt to perform PCA on the point cloud given the indices
        pca.fit(cloud_indices)
        eigen_vectors = pca.components_ # Extract principle axes
        rotation_matrix = eigen_vectors.T # 
        translation_vector = np.mean(cloud_indices, axis=0)
        rot_vector = Rotation.from_matrix(rotation_matrix).as_quat()
        Point_pose = Point()
        Point_pose.x, Point_pose.y, Point_pose.z = translation_vector[0], translation_vector[1], translation_vector[2]
        
        Rot_pose = Quaternion()
        Rot_pose.x, Rot_pose.y, Rot_pose.z, Rot_pose.w = rot_vector[0], rot_vector[1], rot_vector[2], rot_vector[3]
        # compute pose from Principal Component Analysis of object point cloud
        object_pose=PoseStamped()
        object_pose.header=header
        object_pose.pose.position = Point_pose
        object_pose.pose.orientation = Rot_pose
        try:
            
            return object_pose
        except:
            return
        

    def convert_detection_msg_to_bounding_box(self,bounding_box_msg):
        try:
            bounding_box=(bounding_box_msg.xmin, bounding_box_msg.ymin, bounding_box_msg.xmax, bounding_box_msg.ymax)
            return bounding_box
        except:
            return None
        
    #####################
    # Callback methods  #
    #####################

    # Depth image callback (sensors_msgs/Image)
    def camera_depth_callback(self,img_depth):
        try:
            self.depth_header = img_depth.header
            self.depth_image = self.bridge.imgmsg_to_cv2(img_depth, desired_encoding='passthrough')
            #print(self.depth_image)
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
            objects_poses_array_msg=Path()
            objects_poses_array_msg.header=self.depth_header
            for bounding_box_msg in detections_msg.bounding_boxes:
                # convert bounding box to (x_min,y_min,x_max,y_max) format
                bounding_box=self.convert_detection_msg_to_bounding_box(bounding_box_msg)

                # extract object point cloud from detection bounding box and depth image
                object_point_cloud=self.convert_depth_image_to_pointcloud(self.depth_image, bounding_box)
                
                # compute 6D pose using point_cloud information
                pose=self.compute_pose(object_point_cloud, self.depth_header)

                # add pose msg to poses array msg
                objects_poses_array_msg.poses.append(pose)

            # publish poses
            # print(objects_poses_array_msg)
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