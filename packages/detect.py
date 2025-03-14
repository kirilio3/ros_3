#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo

import cv2
import numpy as np
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        # Static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._camera_info_topic = f"/{self._vehicle_name}/camera_node/camera_info"
        self._undistorted_topic = f"/{self._vehicle_name}/camera_node/image/distorted_image/compressed"

        # Bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # Variables to store camera matrix and distortion coefficients
        self._camera_matrix = None
        self._distortion_coeffs = None

        # HSV range for yellow (tune these values based on your environment)
        self._yellow_lower = np.array([20, 100, 100], np.uint8)
        self._yellow_upper = np.array([30, 255, 255], np.uint8)

        # HSV range for white (tune these values based on your environment)
        self._white_lower = np.array([0, 0, 200], np.uint8)
        self._white_upper = np.array([180, 30, 255], np.uint8)

        # Kernel for morphological operations
        # self._kernel = np.ones((5, 5), "uint8")

        # Camera height above the lane (in meters) - adjust this based on your setup
        self._camera_height = 0.1  

        # Subscribers
        self.camera_info_sub = rospy.Subscriber(self._camera_info_topic, CameraInfo, self.camera_info_callback)
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        # Publisher for the processed image
        self.image_pub = rospy.Publisher(self._undistorted_topic, CompressedImage, queue_size=10)

    def camera_info_callback(self, msg):
        # Extract camera matrix (K) and distortion coefficients (D)
        self._camera_matrix = np.array(msg.K).reshape(3, 3)
        self._distortion_coeffs = np.array(msg.D)
        # rospy.loginfo("Received camera info")

    def detect_lines_and_lane_width(self, image):
        
        if self._camera_matrix is None:
            rospy.logwarn("Camera matrix not available yet, cannot compute real-world width.")
            return image, None

        # Focal length in pixels (fx from camera matrix)
        focal_length = self._camera_matrix[0, 0]
        
        # Convert to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create masks for yellow and white
        yellow_mask = cv2.inRange(hsv_image, self._yellow_lower, self._yellow_upper)
        # yellow_mask = cv2.dilate(yellow_mask, self._kernel)

        white_mask = cv2.inRange(hsv_image, self._white_lower, self._white_upper)
        # white_mask = cv2.dilate(white_mask, self._kernel)

        # Find contours for yellow
        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        yellow_inner_x = None
        yellow_y = None  # To approximate vertical position
        for contour in yellow_contours:
            area = cv2.contourArea(contour)
            if area > 300:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(image, "Yellow Line", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                # Inner edge of yellow line is the rightmost point (x + w)
                if yellow_inner_x is None or (x + w) > yellow_inner_x:
                    yellow_inner_x = x + w
                    yellow_y = y + h // 2  # Approximate vertical center of the contour

        # Find contours for white
        white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        white_inner_x = None
        white_y = None  # To approximate vertical position
        for contour in white_contours:
            area = cv2.contourArea(contour)
            if area > 300:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.putText(image, "White Line", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                # Inner edge of white line is the leftmost point (x)
                if white_inner_x is None or x < white_inner_x:
                    white_inner_x = x
                    white_y = y + h // 2

        # Calculate lane width in meters if both lines are detected
        lane_width_meters = None
        lane_center_x = None
        if yellow_inner_x is not None and white_inner_x is not None:
            # Pixel distance between inner edges
            pixel_width = abs(white_inner_x - yellow_inner_x)
            # Convert to meters: (pixel_width * real-world distance) / focal_length
            lane_width_meters = (pixel_width) / focal_length
            # Display lane width on the image
            # Calculate lane center (midpoint between inner edges)
            lane_center_x = (yellow_inner_x + white_inner_x) // 2
            # Approximate vertical center (average of yellow and white y-positions)
            lane_center_y = (yellow_y + white_y) // 2

            # Display lane width on the image
            width_text = f"Lane Width: {lane_width_meters:.2f} m"
            width_text_size = cv2.getTextSize(width_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            width_text_x = (image.shape[1] - width_text_size[0]) // 2
            width_text_y = image.shape[0] - 20
            cv2.putText(image, width_text, (width_text_x, width_text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Mark the lane center with a circle and label
            cv2.circle(image, (lane_center_x, lane_center_y), 5, (0, 0, 255), -1)  # Red dot
            cv2.putText(image, "Lane Center", (lane_center_x + 10, lane_center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        return image, lane_width_meters, lane_center_x
        #     text = f"Lane Width: {lane_width_meters:.2f} m"
        #     text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        #     text_x = (image.shape[1] - text_size[0]) // 2
        #     text_y = image.shape[0] - 20  # Near the bottom
        #     cv2.putText(image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # return image, lane_width_meters

    def callback(self, msg):
        if self._camera_matrix is None or self._distortion_coeffs is None:
            rospy.logwarn("Waiting for camera calibration parameters...")
            return
        
        # Convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        # Undistort the image
        undistorted_image = cv2.undistort(image, self._camera_matrix, self._distortion_coeffs)

        # Detect lines and calculate lane width in meters
        processed_image, lane_width_meters, lane_center_x = self.detect_lines_and_lane_width(undistorted_image)

        # Log the results (optional)
        # if lane_width_meters is not None and lane_center_x is not None:
        #     rospy.loginfo(f"Lane width: {lane_width_meters:.2f} meters, Lane center x: {lane_center_x} pixels")

        # # Log the lane width (optional)
        # if lane_width_meters is not None:
        #     rospy.loginfo(f"Lane width: {lane_width_meters:.2f} meters")

        # Convert back to ROS message and publish
        undistorted_msg = self._bridge.cv2_to_compressed_imgmsg(processed_image)
        self.image_pub.publish(undistorted_msg)

if __name__ == '__main__':
    # Create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # Keep spinning
    rospy.spin()

