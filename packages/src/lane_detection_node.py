#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import math
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class LaneDetectionNode(DTROS):
    def __init__(self, node_name):
        # Initialize the node as a visualization type within Duckietown framework
        super(LaneDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        # Get vehicle name from environment variable
        self._vehicle_name = os.environ['VEHICLE_NAME']
        # Define camera topic based on vehicle name
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        
        # Initialize CvBridge for converting ROS images to OpenCV format
        self._bridge = CvBridge()
        
        # Set up subscriber for camera images and publisher for processed images
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self.pub = rospy.Publisher(f"/{self._vehicle_name}/lane_detection/image/compressed", CompressedImage, queue_size=10)
        
        # Initialize variables to track lane length changes over time
        self.lane_lengths = []  # List to store decreases in lane length
        self.previous_lane_length = None  # Store the last detected lane length
        self.start_time = time.time()  # Timestamp for periodic reset of lane_lengths
    
    def callback(self, msg):
        # Process incoming camera image messages
        image = self._bridge.compressed_imgmsg_to_cv2(msg)  # Convert ROS compressed image to OpenCV format
        processed_image, lane_length, lane_width = self.detect_lane(image)  # Detect lane and get measurements
        output_msg = self._bridge.cv2_to_compressed_imgmsg(processed_image)  # Convert back to ROS message
        self.pub.publish(output_msg)  # Publish processed image
        
        if lane_length:
            # Track changes in lane length if a previous value exists
            if self.previous_lane_length is not None:
                lane_length_decrease = self.previous_lane_length - lane_length
                self.lane_lengths.append(lane_length_decrease)  # Record decrease as robot approaches lane
            self.previous_lane_length = lane_length  # Update previous length
        
        # Reset lane_lengths list every 5 seconds to avoid memory buildup
        if time.time() - self.start_time >= 5:
            self.start_time = time.time()
            self.lane_lengths.clear()
    
    def detect_lane(self, image):
        # Convert image to HSV color space for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define HSV ranges for yellow lane lines
        lower_yellow = np.array([20, 100, 100], np.uint8)
        upper_yellow = np.array([30, 255, 255], np.uint8)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)  # Mask for yellow regions

        # Define HSV ranges for white lane lines
        lower_white = np.array([0, 0, 200], np.uint8)
        upper_white = np.array([180, 50, 255], np.uint8)
        white_mask = cv2.inRange(hsv, lower_white, upper_white)  # Mask for white regions

        # Combine yellow and white masks to detect all lane lines
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        edges = cv2.Canny(combined_mask, 50, 150)  # Detect edges in the combined mask
        
        # Use Hough Transform to detect lines in the edge image
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=10)

        # Lists to store coordinates of left and right lane lines
        left_x, right_x, top_y, bottom_y = [], [], [], []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Classify points as left or right based on image center
                if x1 < image.shape[1] // 2 and x2 < image.shape[1] // 2:
                    left_x.extend([x1, x2])
                elif x1 > image.shape[1] // 2 and x2 > image.shape[1] // 2:
                    right_x.extend([x1, x2])
                top_y.extend([y1, y2])  # Collect y-coordinates for top
                bottom_y.extend([y1, y2])  # Collect y-coordinates for bottom
        
        if left_x and right_x and top_y and bottom_y:
            # Determine bounding box for the detected lane
            min_x, max_x = max(left_x), min(right_x)  # Leftmost right line and rightmost left line
            min_y, max_y = min(top_y), max(bottom_y)  # Top and bottom of the lane (Note: 'custom_y' seems to be a typo)

            # Camera parameters for distance estimation
            focal_length = 900  # Focal length in pixels (tunable)
            cx, cy = image.shape[1] // 2, image.shape[0] // 2  # Image center
            
            # Estimate depth (Z) at top and bottom of lane
            Z_min = focal_length / (max_y)  # Depth at bottom
            Z_max = focal_length / (max_y - cy)  # Depth at top (adjusted by center)
            
            # Calculate pixel distances for length and width
            dx_pixels, dy_pixels = max_x - min_x - 10, max_y - min_y - 10  # Subtract 10 for margin
            pixel_distance = math.sqrt(dx_pixels**2 + dy_pixels**2)  # Diagonal distance in pixels

            # Convert pixel measurements to meters
            lane_length_meters = pixel_distance * Z_max / focal_length  # Length along the lane
            lane_width_meters = dx_pixels * Z_min / focal_length  # Width across the lane
            
            # Draw bounding box and text on the image
            cv2.rectangle(image, (min_x, min_y), (max_x, max_y), (0, 255, 0), 3)  # Green rectangle
            cv2.putText(image, f"Lane Length: {lane_length_meters:.2f} m", (min_x, min_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)  # Length label
            cv2.putText(image, f"Lane Width: {lane_width_meters:.2f} m", (min_x, min_y - 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)  # Width label
            
            return image, lane_length_meters, lane_width_meters
        
        # Return image with no measurements if no lane is detected
        return image, None, None

if __name__ == '__main__':
    node = LaneDetectionNode(node_name='lane_detection_node')
    rospy.spin()  