#!/usr/bin/env python3

import rospy, cv2
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image, CameraInfo # For later implementation of tracking positions
from std_msgs.msg import String # For later implementation of tracking positions
from cv_bridge import CvBridge, CvBridgeError
from math import pi, pow

class object_detector_2d():

    def __init__(self, image_sub: str = '/camera/color/image_rect', image_pub: str = '/detection_image', object_pub: str = '/detected_objects', bound_width: int = 2) -> None:
        """
        """
        self._BOUND_WIDTH = bound_width
        self._image_sub = rospy.Subscriber(f'{image_sub}', Image, callback=self.detect_objects)
        #self._detected_pub = rospy.Publisher(f'{object_pub}', list, queue_size=5)
        self._image_pub = rospy.Publisher(f'{image_pub}', Image, queue_size=5)

        # Setup bridge
        self.bridge = CvBridge()
        
        # Define color spaces
        self.blue_color_low = np.array([110, 50, 50])
        self.blue_color_high = np.array([139, 255, 255])
    
        self.green_color_low = np.array([45, 50, 50])
        self.green_color_high = np.array([70, 255, 255])
    
        self.yellow_color_low = np.array([26, 50, 50])
        self.yellow_color_high = np.array([36, 255, 255])
    
        self.red_color_low = np.array([0, 50, 50])
        self.red_color_high = np.array([10, 255, 255])
        self.redn_color_low = np.array([170, 50, 50])
        self.redn_color_high = np.array([180, 255, 255])
        
        rospy.init_node('object_detection', anonymous=False)
        
        rospy.spin()
        
    def detect_objects(self, msg) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Conversion error: {}".format(e))
            return
    
        # Convert image to HSV colorspace for easier detection and color separation
        hsv_image = cv2.cvtColor(image, cv.COLOR_BGR2HLS)
        grey_image = cv2.cvtColor(image, cv.COLOR_BGR2GRAY)
        
        # Detect edges with Canny edge detetor
        edges = cv2.Canny(grey_image, 50, 150, 3)
        
        # Find contours of objects in the image
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        # Find cubes and spheres by contours
        # Instantiate empty lists for keeping track of the objects
        cubes = []
        cube_color = []
        spheres = []
        sphere_color = []
        detections = np.empty

        for contour in contours:
            # Reduce the contours into line segments
            detection = cv.approxPolyDP(contour, 0.02*cv.arcLength(contour, True), True)
            
            # Create a minimum area bounding box for comparrisons
            min_rect = cv.minAreaRect(contour)

            # Create a minimum enclosing circle for comparrisons
            min_circ = cv.minEnclosingCircle(contour)
            
            # Separate the HSV-image, for checking hue in every contour
            h, s, v = hsv_image[:, :, 0], hsv_image[:, :, 1], hsv_image[:, :, 2]

            # Assuming that boxes consist of 4 line segments and should at least be occupying 90% of the area as the bounding box
            if len(detection) == 4 and cv.contourArea(contour) > 0.9 * min_rect[1][0] * min_rect[1][1]:
                cubes.append(detection)
                
                # Check that what color the center of the bounding box is
                if (self.blue_color_low[0] <= h[int(min_rect[0][1]), int(min_rect[0][0])]) and (h[int(min_rect[0][1]), int(min_rect[0][0])] <= self.blue_color_high[0]):
                    cube_color.append('blue')
                elif (self.green_color_low[0] <= h[int(min_rect[0][1]), int(min_rect[0][0])]) and (h[int(min_rect[0][1]), int(min_rect[0][0])] < self.green_color_high[0]):
                    cube_color.append('green')
                elif (self.yellow_color_low[0] <= h[int(min_rect[0][1]), int(min_rect[0][0])]) and (h[int(min_rect[0][1]), int(min_rect[0][0])] < self.yellow_color_high[0]):
                    cube_color.append('yellow')
                elif (self.red_color_low[0] <= h[int(min_rect[0][1]), int(min_rect[0][0])]) and (h[int(min_rect[0][1]), int(min_rect[0][0])] < self.red_color_high[0]) or (self.redn_color_low[0] <= h[int(min_rect[0][1]), int(min_rect[0][0])]) and (h[int(min_rect[0][1]), int(min_rect[0][0])] <= self.redn_color_high[0]):
                    cube_color.append('red')
                else:
                    cube_color.append('Unknown')
                    
            # Assuming that the area of a sphere/circle would correspond to the minimal enclosing circle area
            if cv.contourArea(contour) > 0.9 * pi * pow(min_circ[1], 2):
                spheres.append(detection)
                # Check that what color the center of the enclosing circle is
                if (self.blue_color_low[0] <= h[int(min_circ[0][1]), int(min_circ[0][0])]) and (h[int(min_circ[0][1]), int(min_circ[0][0])] < self.blue_color_high[0]):
                    sphere_color.append('blue')
                elif (self.green_color_low[0] <= h[int(min_circ[0][1]), int(min_circ[0][0])]) and (h[int(min_circ[0][1]), int(min_circ[0][0])] < self.green_color_high[0]):
                    sphere_color.append('green')
                elif (self.yellow_color_low[0] <= h[int(min_circ[0][1]), int(min_circ[0][0])]) and (h[int(min_circ[0][1]), int(min_circ[0][0])] < self.yellow_color_high[0]):
                    sphere_color.append('yellow')
                elif ((self.red_color_low[0] <= h[int(min_circ[0][1]), int(min_circ[0][0])]) and (h[int(min_circ[0][1]), int(min_circ[0][0])] < self.red_color_high[0])) or ((self.redn_color_low[0] <= h[int(min_circ[0][1]), int(min_circ[0][0])])and (h[int(min_circ[0][1]), int(min_circ[0][0])] <= self.redn_color_high[0])):
                    sphere_color.append('red')
                else:
                    sphere_color.append('Unknown')

        # Copy the input image
        image_detections = image.copy()
        i = 0
        
        # Draw bounding boxes for cubes
        for cube in cubes:
            cv.drawContours(image_detections, [cube], 0, (0,0,255), self._BOUND_WIDTH)
            cv.putText(image_detections, f'{cube_color[i]} cube{i}', (cube[0][0][0], cube[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            i = i + 1
        i = 0
        # Draw circles for spheres
        for sphere in spheres:
            cv.drawContours(image_detections, [sphere], 0, (0,0,255), self._BOUND_WIDTH)
            cv.putText(image_detections, f'{sphere_color[i]} sphere{i}', (sphere[0][0][0], sphere[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
            i = i +1
        
        # Publish the result
        self._image_pub.publish(self.bridge.cv2_to_imgmsg(image_detections))



if __name__ == "__main__":
    obj_detect = object_detector_2d() 



