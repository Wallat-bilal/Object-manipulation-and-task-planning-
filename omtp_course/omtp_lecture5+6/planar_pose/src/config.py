from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import cv2

# Reference frame for pose estimation
frame_id = 'camera_depth_frame'

# Image topic for object detection
image_sub = {
    'topic': 'camera/depth/image_rect',
    'type': Image
    }

# Pointcloud topic for retrieving
# 3D locations
pc_sub = {
    'topic': '/camera/depth_registered/points_xyz',
    'type': PointCloud2
    }

to_gray = True

# if hold_prev_value is True, then it will
# hold the previous value if no value is
# recieved from the method for hold_period
# amount of time
hold_prev_vals = True
if hold_prev_vals:
    hold_period = 5  # in seconds

# CameraInfo topic for retrieving
# camera matrix

cam_info_sub = {
    'topic': '/camera/registered/camera_info',
    'type': CameraInfo
    }

# Make sure all the topics corrsponds
# image resolution, i.e. qhd, sd, or hd
assert image_sub['topic'].split('/')[2] == pc_sub['topic'].split('/')[2], 'image topic and point cloud topic have resolution mismatch'  # noqa: E501
assert image_sub['topic'].split('/')[2] == cam_info_sub['topic'].split('/')[2], 'image topic and camera info topic have resolution mismatch'  # noqa: E501


# Path containing images of query objects
object_path = 'objects'
# Name of the objects corresponding to
# the files name without the extension
# e.g. book-1.jpg corresponds to book-1
objects = ['ball', 'box']

# Minimum match required for an object to be considered detected
min_match_count = 4

# Define detector-descriptor and the
# matcher algortihm
detector_descriptor = cv2.SIFT_create()
matcher = cv2.FlannBasedMatcher(dict(algorithm=1, trees=5),
                                dict(checks=50)).knnMatch

# If there's no kwargs for the matcher keep
# it as an empty `dict` e.g. {}
matcher_kwargs = {'k': 2}

# Visualize the detected objects,
# if set to True
show_image = True

# Visualize the pose of the objects,
# if set to True
viz_pose = True
