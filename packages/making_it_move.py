#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float64 as Float
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, LEDPattern
import signal
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, CameraInfo

class DShapeNode(DTROS):

    def __init__(self, node_name):
        super(DShapeNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # Get the Duckiebot name from environment
        self.vehicle_name = os.environ['VEHICLE_NAME']
        # Publisher for velocity commands
        twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        
        # Topic to get info about camera calibrations
        self._camera_info_topic = f"/{self.vehicle_name}/camera_node/camera_info"

        # Publisher for LED control
        self.led_topic = f"/{self.vehicle_name}/led_emitter_node/led_pattern"
        self.led_pub = rospy.Publisher(self.led_topic, LEDPattern, queue_size=1)

        # Subscribe to left and right wheel encoder topics
        self.left_encoder_topic  = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
        self.right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"

        self.sub_left_enc  = rospy.Subscriber(self.left_encoder_topic,  WheelEncoderStamped, self.cb_left_encoder)
        self.sub_right_enc = rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.cb_right_encoder)

        # Subscriber for camera_info intrinsic parameters
        self.camera_info_sub = rospy.Subscriber(self._camera_info_topic, CameraInfo, self.camera_info_callback)
        
        # Subscriber for camera image
        self._camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self.sub_camera = rospy.Subscriber(self._camera_topic, CompressedImage, self.cb_camera)
        self._bridge = CvBridge()

        # Variables to store camera matrix and distortion coefficients
        self._camera_matrix = None
        self._distortion_coeffs = None

        # Variables to store the last tick count (None until we get first reading)
        self.last_left_ticks  = None
        self.last_right_ticks = None

        # ---- TUNABLE PARAMETERS ----
        self.TICKS_PER_REV = 135            # typical for Duckietown wheel encoder
        self.WHEEL_RADIUS  = 0.0318         # ~3.18 cm radius
        self.WHEEL_CIRC    = 2.0 * math.pi * self.WHEEL_RADIUS  # circumference in meters
        self.BASELINE      = 0.077            # distance between wheels in meters (approx)

        self.TOL = 0.08     

        # Desired angles (radians)
        self.three_sixty = math.pi                  # 360 degrees
        self.one_eighty = self.three_sixty / 2      # 180 degrees
        self.ninety = self.three_sixty / 4          # 90 degrees

        # Angular speed (rad/s) for in-place rotation
        self.OMEGA_SPEED = 10  # tune me
        self.angular_vel = 2.6

        self.VELOCITY = 0.4  # m/s
        self.VELOCITY_LEFT = 0.4  # Adjust for left wheel if needed
        self.VELOCITY_RIGHT = 0.4  # Slightly adjust right wheel to compensate

        self.small_tune = 0.2    # no rotation, going straight

        self.turn_duration = (self.ninety * (math.pi / (self.one_eighty))) / self.angular_vel

        self._left_distance_traveled = 0.0
        self._right_distance_traveled = 0.0
        
        # Wheel/encoder parameters (these may vary for your Duckiebot)
        self.TICKS_PER_REV = 135    # typical for standard Duckietown wheel encoders
        self.WHEEL_CIRCUM  = 2.0 * math.pi * self.WHEEL_RADIUS  # meters per revolution
        
        # Target distances
        self.FORWARD_DISTANCE  = 1.25
        self.BACKWARD_DISTANCE = -1.25

        # Register signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        # Green line detection flag
        self.green_line_reached = False

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

    def camera_info_callback(self, msg):
        # Extract camera matrix (K) and distortion coefficients (D)
        self._camera_matrix = np.array(msg.K).reshape(3, 3)
        self._distortion_coeffs = np.array(msg.D)

    def cb_camera(self, msg):
        if self._camera_matrix is None or self._distortion_coeffs is None:
            rospy.logwarn("Waiting for camera calibration parameters...")
            return
        
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # Undistort the image using the camera calibration parameters
        image = cv2.undistort(image, self._camera_matrix, self._distortion_coeffs)

        
        processed_image, green_distance = self.detect_green_line(image)
        # processed_image, lane_length, lane_width = self.detect_lane(processed_image)

        if green_distance is None:
            self.green_line_reached = True
            rospy.loginfo("Green line reached, stopping the robot.")
            self.stop()

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

    def straight_line(self, distance):
        rospy.loginfo(f"Moving forward {distance} meters...")
        
        # Set omega to 0 for straight motion (no angular velocity)
        forward_msg = Twist2DStamped(v=self.VELOCITY, omega=0.0)
        self.set_led_color('GREEN')  # Set LED color to GREEN when moving straight
        self.pub_cmd.publish(forward_msg)
        rate = rospy.Rate(10)
        
        # While moving, keep track of the distance traveled
        while not rospy.is_shutdown() and not self.green_line_reached:
            heading_error = self.compute_heading()  # Get heading error
            if abs(heading_error) > 0.05:  # Threshold for deviation
                correction = 0.1 * heading_error  # Simple proportional control
                forward_msg = Twist2DStamped(v=self.VELOCITY, omega=-correction)
                self.pub_cmd.publish(forward_msg)
            if (self._right_distance_traveled + self._left_distance_traveled) / 2 >= distance:
                self._right_distance_traveled = 0.0
                self._left_distance_traveled = 0.0
                self.last_left_ticks = None
                self.last_right_ticks = None
                self.stop()
                break
            self.pub_cmd.publish(forward_msg)
            rate.sleep()

    def set_led_color(self, color):
        """ Helper function to publish LED color """
        pattern = LEDPattern()
        if color == 'CYAN':
            selected_color = ColorRGBA(0.0, 1.0, 1.0, 1.0)  # Cyan color
        elif color == 'GREEN':
            selected_color = ColorRGBA(0, 1, 0, 1)  # Green color
        elif color == 'RED':
            selected_color = ColorRGBA(1, 0, 0, 1)  # Red color    
        else:  # Default PURPLE
            selected_color = ColorRGBA(0.5, 0.0, 0.5, 1.0)  # Purple color
        
        pattern.color_list = [color] * 5  # Apply color to all LEDs
        pattern.rgb_vals = [selected_color] * 5  # Set the color
        pattern.color_mask = [1, 1, 1, 1, 1]  # Affect all LEDs
        pattern.frequency = 1.0  # Optional: Blinking speed
        pattern.frequency_mask = [1, 1, 1, 1, 1]  # Apply frequency to all LEDs
        self.led_pub.publish(pattern)

    def stop(self):
        msg = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd.publish(msg)

    def on_shutdown(self):
        rospy.loginfo("Shutting down node...")
        self.stop()  # Stop robot before shutdown
        super(DShapeNode, self).on_shutdown()

    def signal_handler(self, sig, frame):
        """Handle the signal from Ctrl+C to terminate the node gracefully."""
        rospy.loginfo("Ctrl+C detected, shutting down...")
        self.on_shutdown()  # Call shutdown procedure
        sys.exit(0)  # Exit the program

    def compute_heading(self):
        return (self._right_distance_traveled - self._left_distance_traveled) / self.BASELINE

    def run(self):
        rospy.sleep(2.0)
        self.straight_line(1.3)
        
        
if __name__ == "__main__":
    node = DShapeNode(node_name="d_shape_node")
    node.run()
    rospy.loginfo("DShapeNode main finished.")

