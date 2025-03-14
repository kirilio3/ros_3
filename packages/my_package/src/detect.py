#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped  # Make sure this is the correct message type for your setup
import cv2
import numpy as np
from cv_bridge import CvBridge
import math

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        # Static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._camera_info_topic = f"/{self._vehicle_name}/camera_node/camera_info"
        self._undistorted_topic = f"/{self._vehicle_name}/camera_node/image/distorted_image/compressed"


        self.left_encoder_topic  = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self.right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"

        self.sub_left_enc  = rospy.Subscriber(self.left_encoder_topic,  WheelEncoderStamped, self.cb_left_encoder)
        self.sub_right_enc = rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.cb_right_encoder)

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

        # Camera height above the lane (in meters) - adjust this based on your setup
        self._camera_height = 0.1  

        # Subscribers for camera info and image
        self.camera_info_sub = rospy.Subscriber(self._camera_info_topic, CameraInfo, self.camera_info_callback)
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        

        # Publisher for the processed image
        self.image_pub = rospy.Publisher(self._undistorted_topic, CompressedImage, queue_size=10)

        # Publisher for velocity commands (using Twist2DStamped)
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)


        self.last_left_ticks  = None
        self.last_right_ticks = None
        self._left_distance_traveled = 0.0
        self._right_distance_traveled = 0.0
        self.TICKS_PER_REV = 135            # typical for Duckietown wheel encoder
        self.WHEEL_RADIUS  = 0.0318         # ~3.18 cm radius
        self.WHEEL_CIRC    = 2.0 * math.pi * self.WHEEL_RADIUS  # circumference in meters
        self.BASELINE      = 0.1016            # distance between wheels in meters (approx)


        self.sign = 0
        # Controller parameters (you can set these via ROS params or adjust defaults)
        self.kp = 0.1
        self.kd = 0.001
        self.ki = 0.0001
        self.controller_type = "P"  # "P", "PD", or "PID"
        self.prev_error = 0.0
        self.integral = 0.0
        self.bias = 0.05
        self.steer = 0.0
        self.VELOCITY = 0.2
        rospy.loginfo("CameraReaderNode initialized, waiting for camera info and encoder messages...")

    def camera_info_callback(self, msg):
        # Extract camera matrix (K) and distortion coefficients (D)
        self._camera_matrix = np.array(msg.K).reshape(3, 3)
        self._distortion_coeffs = np.array(msg.D)



    def detect_lines_and_lane_width(self, image):
    
        if self._camera_matrix is None:
            rospy.logwarn("Camera matrix not available yet, cannot compute real-world width.")
            return image, None, None

        height, width, _ = image.shape
        
        # Define ROI for the lower half of the image (adjust as needed)
        roi_y_start = 0
        roi = image[roi_y_start:height, :]  # Using full width for now
        
        # Focal length in pixels (fx from the camera matrix)
        focal_length = self._camera_matrix[0, 0]
        
        # Convert ROI to HSV color space
        hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Create masks for yellow and white
        yellow_mask = cv2.inRange(hsv_image, self._yellow_lower, self._yellow_upper)
        white_mask = cv2.inRange(hsv_image, self._white_lower, self._white_upper)

        # Find contours for yellow line
        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        yellow_inner_x = None
        yellow_y = None  # Approximate vertical position within ROI
        for contour in yellow_contours:
            area = cv2.contourArea(contour)
            if area > 300:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y + roi_y_start), (x + w, y + h + roi_y_start), (0, 255, 255), 2)
                cv2.putText(image, "Yellow Line", (x, y + roi_y_start - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                if yellow_inner_x is None or (x + w) > yellow_inner_x:
                    yellow_inner_x = x + w
                    yellow_y = y + h // 2

        # Find contours for white line
        white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        white_inner_x = None
        white_y = None  # Approximate vertical position within ROI
        for contour in white_contours:
            area = cv2.contourArea(contour)
            if area > 300:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y + roi_y_start), (x + w, y + h + roi_y_start), (255, 255, 255), 2)
                cv2.putText(image, "White Line", (x, y + roi_y_start - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                if white_inner_x is None or x < white_inner_x:
                    white_inner_x = x
                    white_y = y + h // 2

        lane_width_meters = None
        lane_center_x = None
        if yellow_inner_x is not None and white_inner_x is not None:
            pixel_width = abs(white_inner_x - yellow_inner_x)
            # Convert pixel width to meters. A more general formula:
            # lane_width = (pixel_width * Z) / focal_length.
            # Here, we assume an estimated Z distance of 1 meter.
            Z = 1
            lane_width_meters = (pixel_width * Z) / focal_length

            lane_center_x = (yellow_inner_x + white_inner_x) // 2

            lane_center_y = (yellow_y + white_y) // 2 + roi_y_start

            width_text = f"Lane Width: {lane_width_meters-self.bias:.2f} m"
            width_text_size = cv2.getTextSize(width_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            width_text_x = (width - width_text_size[0]) // 2
            width_text_y = height - 20
            cv2.putText(image, width_text, (width_text_x, width_text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.circle(image, (lane_center_x, lane_center_y), 5, (0, 0, 255), -1)
            cv2.putText(image, f"Lane Center {lane_center_x}", (lane_center_x + 10, lane_center_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            image_center_x = width // 2
            rospy.loginfo(f" delta: {image_center_x - lane_center_x}")
            if image_center_x - lane_center_x > 0:
                self.sign = 1
            else:
                self.sign = -1
        return image, lane_width_meters, lane_center_x, image_center_x

    def callback(self, msg):
        if self._camera_matrix is None or self._distortion_coeffs is None:
            rospy.logwarn("Waiting for camera calibration parameters...")
            return
        
        # Convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        # Undistort the image
        undistorted_image = cv2.undistort(image, self._camera_matrix, self._distortion_coeffs)

        # Detect lines and calculate lane width in meters
        processed_image, lane_width_meters, lane_center_x, image_center_x = self.detect_lines_and_lane_width(undistorted_image)

        # Publish the processed image
        undistorted_msg = self._bridge.cv2_to_compressed_imgmsg(processed_image)
        self.image_pub.publish(undistorted_msg)

        # Only drive if the lane is detected and we haven't traveled 1.5 m yet.
        if lane_center_x is not None:
            # Calculate the error (difference between lane center and image center)
            full_width_meter = 0.242
            #error = lane_center_x - (full_width // 2)
            error = (lane_width_meters-self.bias) - (full_width_meter//2)
            rospy.loginfo(f"error:{error}, lane_width_meters:{lane_width_meters}, lane_center_x:{lane_center_x}, image_center_x:{image_center_x}")
            # Choose controller based on parameter
            current_time = rospy.Time.now().to_sec()
            dt = 0.1  # or compute a proper delta time if needed
            if self.controller_type == "P":
                self.steer = self.p_controller(error)
            elif self.controller_type == "PD":
                self.steer = self.pd_controller(error, dt)
            elif self.controller_type == "PID":
                self.steer = self.pid_controller(error, dt)
            else:
                self.steer = 0.0



    def p_controller(self, error):
        """
        Proportional (P) Controller: steering = kp * error
        """
        return self.sign*self.kp * error 

    def pd_controller(self, error, dt):
        """
        Proportional-Derivative (PD) Controller:
        steering = kp * error + kd * (error - previous_error) / dt
        """
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        steer = self.kp * error + self.kd * derivative
        self.prev_error = error
        return steer

    def pid_controller(self, error, dt):
        """
        Proportional-Integral-Derivative (PID) Controller:
        steering = kp * error + kd * derivative + ki * integral
        """
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.integral += error * dt
        steer = self.kp * error + self.kd * derivative + self.ki * self.integral
        self.prev_error = error
        return steer
    


    def cb_left_encoder(self, msg):
        current_ticks = msg.data
        if self.last_left_ticks is None:
            self.last_left_ticks = current_ticks
            return
        delta_ticks = current_ticks - self.last_left_ticks
        if delta_ticks > self.TICKS_PER_REV / 2:
            delta_ticks -= self.TICKS_PER_REV
        elif delta_ticks < -self.TICKS_PER_REV / 2:
            delta_ticks += self.TICKS_PER_REV
        self.last_left_ticks = current_ticks
        distance = (delta_ticks / float(self.TICKS_PER_REV)) * self.WHEEL_CIRC
        self._left_distance_traveled += distance

    def cb_right_encoder(self, msg):
        current_ticks = msg.data
        if self.last_right_ticks is None:
            self.last_right_ticks = current_ticks
            return
        delta_ticks = current_ticks - self.last_right_ticks
        if delta_ticks > self.TICKS_PER_REV / 2:
            delta_ticks -= self.TICKS_PER_REV
        elif delta_ticks < -self.TICKS_PER_REV / 2:
            delta_ticks += self.TICKS_PER_REV
        self.last_right_ticks = current_ticks
        distance = (delta_ticks / float(self.TICKS_PER_REV)) * self.WHEEL_CIRC
        self._right_distance_traveled += distance

    def straight_line(self, distance):
        rospy.loginfo(f"Moving forward {distance} meters...")
        forward_msg = Twist2DStamped(v=self.VELOCITY, omega=self.steer)
        self.pub_cmd.publish(forward_msg)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if (self._right_distance_traveled + self._left_distance_traveled)/2 >= distance:
                self._right_distance_traveled = 0.0
                self._left_distance_traveled = 0.0
                self.last_left_ticks = None
                self.last_right_ticks = None
                self.stop()
                break
            self.pub_cmd.publish(forward_msg)
            rate.sleep()
    def run(self):
        # Keep the node running
        rospy.sleep(1.0)
        self.straight_line(1.5)
        rospy.sleep(1.0)


    def stop(self):
        msg = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd.publish(msg)

    def on_shutdown(self):
        self.stop()
        super(CameraReaderNode, self).on_shutdown()

if __name__ == '__main__':
    node = CameraReaderNode(node_name='camera_reader_node')
    node.run()