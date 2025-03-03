#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge
import numpy as np

class LaneDetectionNode():
    def __init__(self, node_name):
        # super(LaneDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        
        self._bridge = CvBridge()
        
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callbacktest)
        self.pub = rospy.Publisher(f"/{self._vehicle_name}/lane_detection/image/compressed", CompressedImage, queue_size=10)
        self.width_in_meters = 0
        self.height_in_meters = 0

    def callbacktest(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        processed_image = self.detect_lane(image)
        output_msg = self._bridge.cv2_to_compressed_imgmsg(processed_image)
        self.pub.publish(output_msg)
        
    def callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        processed_image = self.detect_lane(image)
        output_msg = self._bridge.cv2_to_compressed_imgmsg(processed_image)
        self.pub.publish(output_msg)
    
    def detect_lane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect yellow lane
        lower_yellow = np.array([20, 100, 100], np.uint8)
        upper_yellow = np.array([30, 255, 255], np.uint8)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Detect white lane
        lower_white = np.array([0, 0, 200], np.uint8)
        upper_white = np.array([180, 50, 255], np.uint8)
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # Combine both masks
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

        # Edge detection
        edges = cv2.Canny(combined_mask, 50, 150)

        # Hough Line Transform to detect lane lines
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=10)

        left_x = []
        right_x = []
        top_y = []
        bottom_y = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                # Classify left and right lane lines
                if x1 < image.shape[1] // 2 and x2 < image.shape[1] // 2:
                    left_x.extend([x1, x2])
                elif x1 > image.shape[1] // 2 and x2 > image.shape[1] // 2:
                    right_x.extend([x1, x2])

                top_y.extend([y1, y2])
                bottom_y.extend([y1, y2])

        if left_x and right_x and top_y and bottom_y:
            min_x = max(left_x)  # Inner side of the left (white) lane
            max_x = min(right_x)  # Inner side of the right (yellow) lane
            min_y = min(top_y)
            max_y = max(bottom_y)

            # Convert pixel coordinates to meters (using camera calibration parameters)
            # Assume the focal length (f) and principal point (cx, cy) are given or calibrated
            focal_length = 800  # Example value in pixels
            camera_height = 0.10  # Camera height in meters (given)
            cx = image.shape[1] // 2  # Principal point x (center of the image)
            cy = image.shape[0] // 2  # Principal point y (center of the image)

            # Calculate depth (Z) for each point
            Z_min = (focal_length * camera_height) / (min_y - cy)  # Depth for min_y
            Z_max = (focal_length ) / (max_y - cy)  # Depth for max_y

            # Calculate real-world width and height of the lane
            self.width_in_meters = (max_x - min_x) * Z_max / focal_length  # Using depth for the near lane line
            self.height_in_meters = (max_y - min_y) * Z_max / focal_length  # Using depth for the near lane line

            # Draw rectangle around the lane (in pixels)
            cv2.rectangle(image, (min_x, min_y), (max_x, max_y), (0, 255, 0), 3)
            cv2.putText(image, f"Width: {self.width_in_meters:.2f} m", (min_x, min_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(image, f"Height: {self.height_in_meters:.2f} m", (min_x, min_y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return image

# if __name__ == '__main__':
#     node = LaneDetectionNode(node_name='lane_detection_node')
#     rospy.spin()
