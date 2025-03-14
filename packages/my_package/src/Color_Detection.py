#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo

import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._camera_info_topic = f"/{self._vehicle_name}/camera_node/camera_info"
        self._blue_topic = f"/{self._vehicle_name}/camera_node/image/blue_line/compressed"
        self._red_topic = f"/{self._vehicle_name}/camera_node/image/red_line/compressed"
        self._green_topic = f"/{self._vehicle_name}/camera_node/image/green_line/compressed"

        # bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # variables to store camera matrix and distortion coefficients
        self._camera_matrix = None
        self._distortion_coeffs = None

        # construct subscriber for camera_info intrinsic parameters
        self.camera_info_sub = rospy.Subscriber(self._camera_info_topic, CameraInfo, self.camera_info_callback)
        
        # construct subscriber for image topics
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        # Publishers for the processed images with different lines
        self.blue_image_pub = rospy.Publisher(self._blue_topic, CompressedImage, queue_size=10)
        self.red_image_pub = rospy.Publisher(self._red_topic, CompressedImage, queue_size=10)
        self.green_image_pub = rospy.Publisher(self._green_topic, CompressedImage, queue_size=10)

    def camera_info_callback(self, msg):
        # Extract camera matrix (K) and distortion coefficients (D)
        self._camera_matrix = np.array(msg.K).reshape(3, 3)
        self._distortion_coeffs = np.array(msg.D)

    def callback(self, msg):
        if self._camera_matrix is None or self._distortion_coeffs is None:
            rospy.logwarn("Waiting for camera calibration parameters...")
            return
        
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        undistorted_image = cv2.undistort(image, self._camera_matrix, self._distortion_coeffs)

        # Draw lines on separate images
        blue_image = undistorted_image.copy()
        red_image = undistorted_image.copy()
        green_image = undistorted_image.copy()
        
        # Increase line thickness to ensure visibility
        cv2.line(blue_image, (50, 240), (590, 240), (255, 0, 0), 60)
        cv2.line(red_image, (50, 240), (590, 240), (0, 0, 255), 60)
        cv2.line(green_image, (50, 240), (590, 240), (0, 255, 0), 60)

        # Convert each processed image with lines back to ROS CompressedImage message
        blue_msg = self._bridge.cv2_to_compressed_imgmsg(blue_image)
        red_msg = self._bridge.cv2_to_compressed_imgmsg(red_image)
        green_msg = self._bridge.cv2_to_compressed_imgmsg(green_image)
        
        # Publish the processed images with different lines
        self.blue_image_pub.publish(blue_msg)
        self.red_image_pub.publish(red_msg)
        self.green_image_pub.publish(green_msg)

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='Color_Detection')
    # keep spinning
    rospy.spin()