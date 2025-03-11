#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import ColorRGBA, Float64 as Float
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, LEDPattern
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge
import signal
import sys

class LaneFollowingNode(DTROS):
    def __init__(self, node_name):
        super(LaneFollowingNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # Topics
        self._camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self._camera_info_topic = f"/{self.vehicle_name}/camera_node/camera_info"
        self._twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
        self._led_topic = f"/{self.vehicle_name}/led_emitter_node/led_pattern"
        self._left_encoder_topic = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"
        self._yellow_lane_topic = f"/{self.vehicle_name}/yellow_lane_position"
        self._white_lane_topic = f"/{self.vehicle_name}/white_lane_position"

        # Publishers
        self.pub_cmd = rospy.Publisher(self._twist_topic, Twist2DStamped, queue_size=1)
        self.led_pub = rospy.Publisher(self._led_topic, LEDPattern, queue_size=1)
        self.yellow_lane_pub = rospy.Publisher(self._yellow_lane_topic, Point, queue_size=1)
        self.white_lane_pub = rospy.Publisher(self._white_lane_topic, Point, queue_size=1)

        # Subscribers
        self.sub_camera = rospy.Subscriber(self._camera_topic, CompressedImage, self.camera_callback)
        self.sub_camera_info = rospy.Subscriber(self._camera_info_topic, CameraInfo, self.camera_info_callback)
        self.sub_left_enc = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.cb_left_encoder)
        self.sub_right_enc = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.cb_right_encoder)

        # CV Bridge
        self._bridge = CvBridge()

        # Camera parameters
        self._camera_matrix = None
        self._distortion_coeffs = None

        # HSV ranges (tune as needed)
        self._yellow_lower = np.array([20, 100, 100], np.uint8)
        self._yellow_upper = np.array([30, 255, 255], np.uint8)
        self._white_lower = np.array([0, 0, 200], np.uint8)
        self._white_upper = np.array([180, 30, 255], np.uint8)

        # Encoder parameters
        self.TICKS_PER_REV = 135
        self.WHEEL_RADIUS = 0.0318
        self.WHEEL_CIRC = 2.0 * math.pi * self.WHEEL_RADIUS
        self.BASELINE = 0.077
        self.last_left_ticks = None
        self.last_right_ticks = None
        self._left_distance_traveled = 0.0
        self._right_distance_traveled = 0.0

        # Control parameters
        self.VELOCITY = 0.3  # Base forward velocity (m/s)
        self.KP = 0.9  # Proportional gain (tune this)
        self.TARGET_DISTANCE = 1.3  # Travel at least 1.5 meters

        # Signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def camera_info_callback(self, msg):
        self._camera_matrix = np.array(msg.K).reshape(3, 3)
        self._distortion_coeffs = np.array(msg.D)

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

    def detect_lanes(self, image):
        if self._camera_matrix is None or self._distortion_coeffs is None:
            rospy.logwarn("Camera parameters not available.")
            return image, None, None

        # Undistort image
        undistorted_image = cv2.undistort(image, self._camera_matrix, self._distortion_coeffs)
        hsv_image = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2HSV)

        # Masks for yellow and white lanes
        yellow_mask = cv2.inRange(hsv_image, self._yellow_lower, self._yellow_upper)
        white_mask = cv2.inRange(hsv_image, self._white_lower, self._white_upper)

        # Detect yellow lane
        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        yellow_inner_x = None
        yellow_y = None
        for contour in yellow_contours:
            area = cv2.contourArea(contour)
            if area > 300:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(undistorted_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
                if yellow_inner_x is None or (x + w) > yellow_inner_x:
                    yellow_inner_x = x + w  # Right edge
                    yellow_y = y + h // 2

        # Detect white lane
        white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        white_inner_x = None
        white_y = None
        for contour in white_contours:
            area = cv2.contourArea(contour)
            if area > 300:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(undistorted_image, (x, y), (x + w, y + h), (255, 255, 255), 2)
                if white_inner_x is None or x < white_inner_x:
                    white_inner_x = x  # Left edge
                    white_y = y + h // 2

        # Publish lane positions
        if yellow_inner_x is not None:
            yellow_msg = Point(x=yellow_inner_x, y=yellow_y, z=0)
            self.yellow_lane_pub.publish(yellow_msg)
        if white_inner_x is not None:
            white_msg = Point(x=white_inner_x, y=white_y, z=0)
            self.white_lane_pub.publish(white_msg)

        return undistorted_image, yellow_inner_x, white_inner_x

    def lane_following_controller(self, yellow_x, white_x, image_x):
        if yellow_x is None or white_x is None:
            rospy.logwarn("One or both lanes not detected, stopping.")
            self.stop()
            return

        # Calculate lane center
        lane_center_x = (yellow_x + white_x) // 2.0
        # rospy.loginfo(lane_center_x)
        image_center_x = image_x   # Assuming 640x480 resolution; adjust as needed
        # rospy.loginfo(image_center_x)
        # Error: difference between image center and lane center
        # error = image_center_x - lane_center_x
        error = lane_center_x - image_center_x
        # rospy.loginfo(error)
        # Proportional control: adjust angular velocity
        omega = 0
        # if error > 0:
        #     omega = self.KP * error / 100 # Scale error to reasonable omega (tune divisor)
        # elif error < 0:
        omega = -self.KP * error / 140 # Scale error to reasonable omega (tune divisor)
        # rospy.loginfo(omega)
        cmd = Twist2DStamped(v=self.VELOCITY, omega=omega)
        self.pub_cmd.publish(cmd)
        self.set_led_color("GREEN")

    def camera_callback(self, msg):
        if self._camera_matrix is None or self._distortion_coeffs is None:
            return
        
        rate = rospy.Rate(100)
        # Convert image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        processed_image, yellow_x, white_x = self.detect_lanes(image)
        image_x = image.shape[0] // 2
        # Lane following control
        avg_distance = (self._left_distance_traveled + self._right_distance_traveled) / 2
        
        if avg_distance >= self.TARGET_DISTANCE:
            rospy.logwarn("Target distance reached, stopping.")
            self.stop()
            return
            # rospy.loginfo("Target distance reached, stopping.")
            # self.stop()
            # return

        # Proceed with lane following if within distance
        self.lane_following_controller(yellow_x, white_x, image_x)

        # Sleep to enforce command rate (e.g., 100 ms)
        rate.sleep()
        # if avg_distance < self.TARGET_DISTANCE:
        #     rate.sleep()
        #     self.lane_following_controller(yellow_x, white_x, image_x)
        # else:
        #     self.stop()
        #     rospy.loginfo("Target distance reached, stopping.")

    def set_led_color(self, color):
        pattern = LEDPattern()
        if color == 'GREEN':
            selected_color = ColorRGBA(0, 1, 0, 1)
        else:
            selected_color = ColorRGBA(0.5, 0, 0.5, 1)  # Default purple
        pattern.color_list = [color] * 5
        pattern.rgb_vals = [selected_color] * 5
        pattern.color_mask = [1, 1, 1, 1, 1]
        self.led_pub.publish(pattern)

    def stop(self):
        msg = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd.publish(msg)
        rospy.sleep(0.1)

        self.set_led_color("PURPLE")
        rospy.sleep(0.1)

    def on_shutdown(self):
        rospy.sleep(0.1)

        # self.set_led_color('OFF')
        # rospy.loginfo("Shutting down node...")
        # self.stop()  # Stop robot before shutdown
        # super(DShapeNode, self).on_shutdown()
        self.stop()
        super(LaneFollowingNode, self).on_shutdown()
        

    def signal_handler(self, sig, frame):
        rospy.loginfo("Ctrl+C detected, shutting down...")
        self.on_shutdown()
        sys.exit(0)
        

if __name__ == "__main__":
    node = LaneFollowingNode(node_name="lane_following_node")
    rospy.spin()

