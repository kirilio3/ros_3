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

"""
to start the gui tool use: dts start_gui_tools csc22911
and after the bot is running use: rqt_image_view

"""
class GreenLineLaneDetectionNode():
    def __init__(self, node_name):
        # super(GreenLineLaneDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        self._vehicle_name = os.environ['VEHICLE_NAME']
        
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        
        self._bridge = CvBridge()
        
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self.pub = rospy.Publisher(f"/{self._vehicle_name}/green_line_lane_detection/image/compressed", CompressedImage, queue_size=10)
        
        self.distances = []
        self.start_time = time.time()
    
    def callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        processed_image, green_distance = self.detect_green_line(image)
        processed_image, lane_length, lane_width = self.detect_lane(processed_image)
        
        output_msg = self._bridge.cv2_to_compressed_imgmsg(processed_image)
        self.pub.publish(output_msg)
        
        if green_distance and green_distance > 20:
            self.distances.append(green_distance)
        
        if time.time() - self.start_time >= 5:
            self.start_time = time.time()
            self.distances.clear()
    
    def average_distance_getter(self):
        if self.distances:
            return sum(self.distances)/len(self.distances)
    
    def detect_green_line(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define green color range
        lower_green = np.array([35, 100, 50], np.uint8)
        upper_green = np.array([85, 255, 255], np.uint8)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        kernel = np.ones((5, 5), np.uint8)
        green_mask = cv2.dilate(green_mask, kernel, iterations=2)

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
                real_height_meters = 0.1  # Estimated real-world height of the green line (adjust if necessary)
                pixel_height = h
                
                if pixel_height > 0:
                    distance = (real_height_meters * focal_length) / pixel_height
                    
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    cv2.putText(image, f"Green Line Distance: {distance:.2f} m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    return image, distance
        
            else:
                # If the line is out of view, assume it's reached
                cv2.putText(image, "Line reached", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                return image, None
        
        # If no green line is detected
        return image, None
    
    def detect_lane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_yellow = np.array([20, 100, 100], np.uint8)
        upper_yellow = np.array([30, 255, 255], np.uint8)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_white = np.array([0, 0, 200], np.uint8)
        upper_white = np.array([180, 50, 255], np.uint8)
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        edges = cv2.Canny(combined_mask, 50, 150)
        
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=10)

        left_x, right_x, top_y, bottom_y = [], [], [], []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x1 < image.shape[1] // 2 and x2 < image.shape[1] // 2:
                    left_x.extend([x1, x2])
                elif x1 > image.shape[1] // 2 and x2 > image.shape[1] // 2:
                    right_x.extend([x1, x2])
                top_y.extend([y1, y2])
                bottom_y.extend([y1, y2])
        
        if left_x and right_x and top_y and bottom_y:
            min_x, max_x = max(left_x), min(right_x)
            min_y, max_y = min(top_y), max(bottom_y)

            focal_length = 50
            cx, cy = image.shape[1] // 2, image.shape[0] // 2
            
            # Calculate distance based on the detected lane width and lane length
            # Z_min = focal_length / (cy)
            Z_max = focal_length / (cy)
            # Z_max = focal_length / (max_y - cy)
            
            dx_pixels, dy_pixels = max_x - min_x, max_y - min_y
            pixel_distance = math.sqrt(dx_pixels**2 + dy_pixels**2)

            lane_length_meters = (dy_pixels * Z_max / focal_length) - 0.2
            # lane_length_meters = pixel_distance * Z_max / focal_length
            lane_width_meters = dx_pixels * Z_max / focal_length
            
            cv2.rectangle(image, (min_x, min_y), (max_x, max_y), (0, 255, 255), 3)
            cv2.putText(image, f"Lane Length: {lane_length_meters:.2f} m", (min_x, min_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(image, f"Lane Width: {lane_width_meters:.2f} m", (min_x, min_y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            return image, lane_length_meters, lane_width_meters
        
        return image, None, None
    

class BlueLineDetectionNodeS(GreenLineLaneDetectionNode):
    def __init__(self, node_name):
        super(BlueLineDetectionNodeS, self).__init__(node_name=node_name)
        
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        
        self._bridge = CvBridge()
        
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self.pub = rospy.Publisher(f"/{self._vehicle_name}/blue_line_detection/image/compressed", CompressedImage, queue_size=10)
        
        self.distances = []
        self.start_time = time.time()
    
    def callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        processed_image, blue_distance = self.detect_blue_line(image)
        
        output_msg = self._bridge.cv2_to_compressed_imgmsg(processed_image)
        self.pub.publish(output_msg)
        
        if blue_distance:
            self.distances.append(blue_distance)
        
        if time.time() - self.start_time >= 5:
            self.start_time = time.time()
            self.distances.clear()
    
def detect_blue_line(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define blue color range
        lower_blue = np.array([100, 150, 0], np.uint8)
        upper_blue = np.array([140, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        kernel = np.ones((5, 5), np.uint8)
        blue_mask = cv2.dilate(blue_mask, kernel, iterations=2)

        edges = cv2.Canny(blue_mask, 50, 150)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Check if the blue line's bounding box is within the field of view
            height, width, _ = image.shape
            if y + h < height and x + w < width and x > 0 and y > 0:
            # Calculate distance only if the line is visible
                focal_length = 50  # Approximate focal length
                real_height_meters = 0.1  # Estimated real-world height of the blue line (adjust if necessary)
                pixel_height = h
            
            if pixel_height > 0:
                distance = (real_height_meters * focal_length) / pixel_height
                
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 3)
                cv2.putText(image, f"Blue Line Distance: {distance:.2f} m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                
                return image, distance
        
            else:
            # If the line is out of view, assume it's reached
                cv2.putText(image, "Line reached", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                return image, None
        
        # If no blue line is detected
        return image, None


if __name__ == '__main__':
    node = GreenLineLaneDetectionNode(node_name='green_line_lane_detection_node')
    rospy.spin()
