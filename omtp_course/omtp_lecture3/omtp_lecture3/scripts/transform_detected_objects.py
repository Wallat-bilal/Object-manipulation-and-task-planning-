#!/usr/bin/env python3
import rospy
from logical_camera.msg import LogicalCameraImage
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

"""
This ROS node subscribes to a logical camera topic and transform the poses of detected objects 
of a specified type into the world frame. It utilizes the TF2 library for transforming object poses from the 
camera frame to the world frame, providing insights into the objects' global positions.

Key Features:
- Subscribes to a logical camera topic to receive detections.
- Filters detections to process only objects of a specified type (e.g., 'box').
- Transforms the pose of each detected object into the world frame using TF2.
- Logs the original pose (in the camera frame) and the transformed pose (in the world frame) for each object.
- Shuts down after processing all detected objects.

Usage:
The script is configured to detect objects named 'box' by default, subscribing to the '/omtp/logical_camera1' topic. 
The camera's frame ID is set to 'logical_camera1_link'. These parameters can be easily adjusted to fit different 
scenarios or camera setups. This script is particularly useful in simulation environments where object detection and 
pose estimation relative to the world frame are required for tasks such as object manipulation.

Dependencies:
- ROS Noetic
- TF2 ROS
- Geometry Messages
- Custom message type from the logical camera plugin

To run this script, execute:
$ rosrun logical_camera transform_object_pose.py

Author: Simon Bøgh
Year: 2024
"""

# Variables for customization
OBJECT_TYPE_TO_DETECT = 'box'  # Type of object to detect and transform
CAMERA_TOPIC = '/omtp/logical_camera1'  # Topic where logical camera data is published
CAMERA_LINK_FRAME_ID = 'logical_camera1_link'  # TF frame ID of the logical camera

def transform_pose(input_pose, target_frame):
    try:
        return tf_buffer.transform(input_pose, target_frame, rospy.Duration(1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Failed to transform pose: %s", str(e))
        return None

def logical_camera_callback(data):
    detected_objects = [model for model in data.models if model.type == OBJECT_TYPE_TO_DETECT]
    if detected_objects:
        for gazebo_object in detected_objects:
            rospy.loginfo(gazebo_object)
            object_pose = PoseStamped()
            object_pose.header.stamp = rospy.Time.now()
            object_pose.header.frame_id = CAMERA_LINK_FRAME_ID
            object_pose.pose = gazebo_object.pose
            
            object_world_pose = transform_pose(object_pose, "world")
            if object_world_pose:
                rospy.loginfo("========================================")
                rospy.loginfo("Pose of \"%s\" in the \"world\" reference frame is: %s", OBJECT_TYPE_TO_DETECT, object_world_pose)
                rospy.loginfo("Pose of \"%s\" in the \"logical camera\" reference frame is: %s", OBJECT_TYPE_TO_DETECT, object_pose)
                rospy.loginfo("Successfully transformed pose.")
                rospy.loginfo("========================================")
    else:
        rospy.loginfo("No '%s' objects detected by the logical camera.", OBJECT_TYPE_TO_DETECT)

    rospy.signal_shutdown('All detected objects have been transformed.')

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.loginfo("Initializing node ...")
    rospy.init_node('transform_object_pose', anonymous=True)

    # Initialize the TF2 buffer and listener. The buffer will store the latest transforms
    # and the listener will listen for new transforms and update the buffer.
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Subscribe to the logical camera topic.
    # The callback function will be called when a new message is received.
    rospy.Subscriber(CAMERA_TOPIC, LogicalCameraImage, logical_camera_callback)

    # Keep the node running until all detected objects have been transformed.
    rospy.spin()
