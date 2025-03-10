#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
import math
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class GreenLineLaneDetectionNode(DTROS):
    def __init__(self, node_name):
        super(GreenLineLaneDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        self._vehicle_name = os.environ['VEHICLE_NAME']
        
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        
        self._bridge = CvBridge()
        
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self.pub = rospy.Publisher(f"/{self._vehicle_name}/green_line_lane_detection/image/compressed", CompressedImage, queue_size=10)
        self._camera_info_topic = f"/{self._vehicle_name}/camera_node/camera_info"

        # Subscriber for camera_info intrinsic parameters
        self.camera_info_sub = rospy.Subscriber(self._camera_info_topic, CameraInfo, self.camera_info_callback)

        self.distances = []

        # Variables to store camera matrix and distortion coefficients
        self._camera_matrix = None
        self._distortion_coeffs = None

        self.start_time = time.time()

        # Flag to indicate if the green line has been reached
        self.green_line_reached = False
    
    def camera_info_callback(self, msg):
        # Extract camera matrix (K) and distortion coefficients (D)
        self._camera_matrix = np.array(msg.K).reshape(3, 3)
        self._distortion_coeffs = np.array(msg.D)

    def callback(self, msg):
        if self._camera_matrix is None or self._distortion_coeffs is None:
            rospy.logwarn("Waiting for camera calibration parameters...")
            return
        
        # Skip processing if the green line has already been reached
        if self.green_line_reached:
            rospy.loginfo("Green line already reached. Skipping processing.")
            return

        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Undistort the image using the camera calibration parameters
        # image = cv2.undistort(image, self._camera_matrix, self._distortion_coeffs)

        processed_image, green_distance = self.detect_green_line(image)
        
        output_msg = self._bridge.cv2_to_compressed_imgmsg(processed_image)
        self.pub.publish(output_msg)
        
        if green_distance:
            self.distances.append(green_distance)
        
        if time.time() - self.start_time >= 5:
            self.start_time = time.time()
            self.distances.clear()
    
    def detect_green_line(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define green color range
        # lower_green = np.array([35, 100, 50], np.uint8)
        # upper_green = np.array([85, 255, 255], np.uint8)
        # lower_green = np.array([100, 50, 50], np.uint8)
        # upper_green = np.array([140, 255, 255], np.uint8)
        lower_green = np.array([100, 50, 20], np.uint8)   # Lower bound for darker blue
        upper_green = np.array([140, 255, 150], np.uint8) # Upper bound for darker blue
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # kernel = np.ones((5, 5), np.uint8)
        # kernel = np.ones((3, 3), np.uint8)
        # green_mask = cv2.dilate(green_mask, kernel, iterations=1)
        # green_mask = cv2.dilate(green_mask, kernel, iterations=2)

        edges = cv2.Canny(green_mask, 50, 150)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Check if the green line's bounding box is within the field of view
            height, width, _ = image.shape
            if y + h < height and x + w < width and x > 0 and y > 0:
                # Calculate distance only if the line is visible
                focal_length = 50  # Approximate focal length
                # focal_length = self._camera_matrix[1, 1]
                real_height_meters = 0.1  # Estimated real-world height of the green line (adjust if necessary)
                pixel_height = h
                
                if pixel_height > 0:
                    # distance = abs((real_height_meters * focal_length) / (pixel_height - (y//2)))
                    distance = (real_height_meters * focal_length) / pixel_height
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    cv2.putText(image, f"Green Line Distance: {distance:.2f} m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    return image, distance
        
            else:
                # If the line is out of view, assume it's reached
                cv2.putText(image, "Line reached", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                self.green_line_reached = True  # Set the flag to True
                return image, None
        
        # If no green line is detected
        return image, None

if __name__ == '__main__':
    node = GreenLineLaneDetectionNode(node_name='green_line_lane_detection_node')
    rospy.spin()
