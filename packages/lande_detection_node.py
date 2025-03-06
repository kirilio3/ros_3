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
        super(LaneDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        
        self._bridge = CvBridge()
        
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self.pub = rospy.Publisher(f"/{self._vehicle_name}/lane_detection/image/compressed", CompressedImage, queue_size=10)
        
        self.lane_lengths = []
        self.previous_lane_length = None
        self.start_time = time.time()
    
    def callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        processed_image, lane_length, lane_width = self.detect_lane(image)
        output_msg = self._bridge.cv2_to_compressed_imgmsg(processed_image)
        self.pub.publish(output_msg)
        
        if lane_length:
            # Track the lane length change over time
            if self.previous_lane_length is not None:
                lane_length_decrease = self.previous_lane_length - lane_length
                self.lane_lengths.append(lane_length_decrease)  # Track the decrease in lane length as the robot moves closer
            self.previous_lane_length = lane_length
        
        if time.time() - self.start_time >= 5:
            self.start_time = time.time()
            self.lane_lengths.clear()
    
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

            focal_length = 650
            cx, cy = image.shape[1] // 2, image.shape[0] // 2
            

            # distance = (real_height_meters * focal_length) / pixel_height
            Z_min = focal_length / (max_y)
            Z_max = focal_length / (max_y - cy)
            
            dx_pixels, dy_pixels = max_x - min_x - 10, max_y - min_y - 10
            pixel_distance = math.sqrt(dx_pixels**2 + dy_pixels**2)

            lane_length_meters = pixel_distance * Z_max / focal_length
            lane_width_meters = dx_pixels * Z_min / focal_length
            
            cv2.rectangle(image, (min_x, min_y), (max_x, max_y), (0, 255, 0), 3)
            cv2.putText(image, f"Lane Length: {lane_length_meters:.2f} m", (min_x, min_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(image, f"Lane Width: {lane_width_meters:.2f} m", (min_x, min_y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            return image, lane_length_meters, lane_width_meters
        
        return image, None, None

if __name__ == '__main__':
    node = LaneDetectionNode(node_name='lane_detection_node')
    rospy.spin()



