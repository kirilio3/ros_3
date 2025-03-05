#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float64 as Float
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, LEDPattern
import signal
import sys
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class DShapeNode(DTROS):

    def __init__(self, node_name):
        super(DShapeNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # Get the Duckiebot name from environment
        self.vehicle_name = os.environ['VEHICLE_NAME']
        # Publisher for velocity commands
        twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        # Publisher for LED control
        self.led_topic = f"/{self.vehicle_name}/led_emitter_node/led_pattern"
        self.led_pub = rospy.Publisher(self.led_topic, LEDPattern, queue_size=1)

        # Subscribe to left and right wheel encoder topics
        self.left_encoder_topic  = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
        self.right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"

        self.sub_left_enc  = rospy.Subscriber(self.left_encoder_topic,  WheelEncoderStamped, self.cb_left_encoder)
        self.sub_right_enc = rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.cb_right_encoder)

        # Lane detection setup
        self._camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self._bridge = CvBridge()
        self.sub_camera = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self.pub_lane_detection = rospy.Publisher(f"/{self.vehicle_name}/lane_detection/image/compressed", CompressedImage, queue_size=10)

        # Store the last tick count (None until we get first reading)
        self.last_left_ticks  = None
        self.last_right_ticks = None

        # Initialize lane metrics
        self.lane_length = None
        self.lane_width = None

        # Set up timers for lane metrics display
        self.timer = rospy.Timer(rospy.Duration(5), self.print_lane_metrics)
        self.start_time = rospy.get_time()

        # ---- TUNABLE PARAMETERS ----
        self.TICKS_PER_REV = 135            # typical for Duckietown wheel encoder
        self.WHEEL_RADIUS  = 0.0318         # ~3.18 cm radius
        self.WHEEL_CIRC    = 2.0 * math.pi * self.WHEEL_RADIUS  # circumference in meters
        self.BASELINE      = 0.077            # distance between wheels in meters (approx)

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

        # Proportional control gain for steering correction
        self.Kp = 0.5  # Proportional gain for steering correction

        # forward_msg = Twist2DStamped(v=self.VELOCITY, omega=0.0)

        
        self.small_tune = 0.2    # no rotation, going straight

        self.turn_duration = (self.ninety * (math.pi / (self.one_eighty))) / self.angular_vel
        # -----------------------------

        self._left_distance_traveled = 0.0
        self._right_distance_traveled = 0.0
        
        # Wheel/encoder parameters (these may vary for your Duckiebot)
        self.TICKS_PER_REV = 135    # typical for standard Duckietown wheel encoders
        self.WHEEL_CIRCUM  = 2.0 * math.pi * self.WHEEL_RADIUS  # meters per revolution
        
        # self.TOL_ANGLE = 0.174533          # 10 degrees in radians
        self.TOL_ANGLE = 0
        self.TOL = 0.08                   # A small tolerance

        # Target distances
        self.FORWARD_DISTANCE  = 1.25
        self.BACKWARD_DISTANCE = -1.25

        self.lane_lengths = []
        self.lane_widths = []
        self.start_time = time.time()

        # Register signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)


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
        
        # Set omega to 0 for straight motion (no angular velocity)
        forward_msg = Twist2DStamped(v=self.VELOCITY, omega=0.0)
        self.set_led_color('GREEN')  # Set LED color to GREEN when moving straight
        self.pub_cmd.publish(forward_msg)
        rate = rospy.Rate(10)
        
        # While moving, keep track of the distance traveled
        while not rospy.is_shutdown():
            # Get the current lane center and compute deviation
            lane_center = self.compute_lane_center()
            if lane_center is not None:
                # Calculate the deviation from the center
                current_position = (self._right_distance_traveled + self._left_distance_traveled) / 2
                deviation = lane_center - current_position
                
                # Apply proportional control to correct the deviation
                if abs(deviation) > 0.05:  # Threshold for deviation
                    correction = self.Kp * deviation  # Proportional control
                    forward_msg.omega = -correction  # Adjust omega to steer back to center
                    self.pub_cmd.publish(forward_msg)
            
            # Check if the desired distance has been traveled
            if (self._right_distance_traveled + self._left_distance_traveled) / 2 >= distance:
                self._right_distance_traveled = 0.0
                self._left_distance_traveled = 0.0
                self.last_left_ticks = None
                self.last_right_ticks = None
                self.stop()
                break
            
            self.pub_cmd.publish(forward_msg)
            rate.sleep()

    def compute_lane_center(self):
        """ Compute the center of the lane based on detected yellow and white lines """
        if self.lane_width is not None:
            return self.lane_width / 2  # Lane center is midpoint of lane length
        return None

    def collect_lane_widths(self):
        start_time = time.time()
        lane_widths = []

        while time.time() - start_time < 5:
            if self.lane_width is not None:
                lane_widths.append(self.lane_width)
            time.sleep(0.1)  # Collect data every 100ms

        if lane_widths:
            self.lane_width = sum(lane_widths) / len(lane_widths)
            rospy.loginfo(f"Computed average lane width: {self.lane_width}")

    def collect_lane_lengths(self):
        start_time = time.time()
        lane_lengths = []

        while time.time() - start_time < 5:
            if self.lane_length is not None:
                lane_lengths.append(self.lane_length)
            time.sleep(0.1)  # Collect data every 100ms

        if lane_lengths:
            self.lane_length = sum(lane_lengths) / len(lane_lengths)
            rospy.loginfo(f"Computed average lane length: {self.lane_length}")
    
    def move_arc_quarter_odometry(self, radius, speed, clockwise=False):

        # 1) Reset odometry, measure initial heading
        rospy.sleep(0.1)  # let odometry callbacks update

        start_heading = self.compute_heading()
        target_change = math.pi / 2.0     # 90 deg

        direction = -1.0 if clockwise else 1.0

        omega = direction * (speed / radius)

        # 3) Start publishing velocity commands
        twist = Twist2DStamped()
        twist.v = speed
        twist.omega = omega
        self.set_led_color('RED')  # Set LED color to CYAN when rotating

        rospy.loginfo(
            f"move_arc_quarter_odometry: radius={radius}, speed={speed}, "
            f"start_heading={start_heading:.2f}, clockwise={clockwise}"
        )

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            current_heading = self.compute_heading()
            delta_heading = current_heading - start_heading

            if clockwise:
                # Check if we've turned about -pi/2
                if (delta_heading <= -((target_change-self.TOL_ANGLE) - self.TOL)):
                    self._right_distance_traveled = 0.0
                    self._left_distance_traveled = 0.0
                    self.last_left_ticks = None
                    self.last_right_ticks = None
                    self.stop()
                    break
            else:
                # Check if we've turned about +pi/2
                if (delta_heading >= ((target_change-self.TOL_ANGLE) - self.TOL)):
                    self._right_distance_traveled = 0.0
                    self._left_distance_traveled = 0.0
                    self.last_left_ticks = None
                    self.last_right_ticks = None
                    self.stop()
                    break

            self.pub_cmd.publish(twist)
            rate.sleep()
        
        self.stop()
        rospy.loginfo("Quarter-circle arc complete (odometry-based).")

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

    def callback(self, msg):
        """ Lane detection callback to process image and detect lanes """
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        processed_image = self.detect_lane(image)
        output_msg = self._bridge.cv2_to_compressed_imgmsg(processed_image)
        self.pub_lane_detection.publish(output_msg)

    def detect_lane(self, image):
        """ Detect lanes and compute metrics like lane length and width """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect yellow and white lanes
        lower_yellow = np.array([20, 100, 100], np.uint8)
        upper_yellow = np.array([30, 255, 255], np.uint8)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_white = np.array([0, 0, 200], np.uint8)
        upper_white = np.array([180, 50, 255], np.uint8)
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # Combine masks
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        edges = cv2.Canny(combined_mask, 50, 150)

        # Detect lane lines using Hough Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=10)

        left_x = []
        right_x = []
        top_y = []
        bottom_y = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Classify lines as left or right lanes
                if x1 < image.shape[1] // 2 and x2 < image.shape[1] // 2:
                    left_x.extend([x1, x2])
                elif x1 > image.shape[1] // 2 and x2 > image.shape[1] // 2:
                    right_x.extend([x1, x2])

                top_y.extend([y1, y2])
                bottom_y.extend([y1, y2])

        if left_x and right_x and top_y and bottom_y:
            min_x = max(left_x)
            max_x = min(right_x)
            min_y = min(top_y)
            max_y = max(bottom_y)

            # Camera parameters (focal length, principal point)
            focal_length = 900
            cx = image.shape[1] // 2
            cy = image.shape[0] // 2

            # Depth calculations (Z-axis)
            Z_min = (focal_length) / (min_y - cy)
            Z_max = (focal_length) / (max_y - cy)

            # Calculate lane length and width
            dx_pixels = max_x - min_x - 5
            dy_pixels = max_y - min_y - 5
            pixel_distance = math.sqrt(dx_pixels**2 + dy_pixels**2)
            self.lane_length = pixel_distance * Z_max / focal_length
            dx_pixels_width = max_x - min_x
            self.lane_width = dx_pixels_width * Z_max / focal_length

            # Draw rectangle around the detected lane
            cv2.rectangle(image, (min_x, min_y), (max_x, max_y), (0, 255, 0), 3)
            cv2.putText(image, f"Lane Length: {self.lane_length:.2f} m", (min_x, min_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(image, f"Lane Width: {self.lane_width:.2f} m", (min_x, min_y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return image
    
    def print_lane_metrics(self, event):
        """ Print lane metrics periodically """
        if self.lane_length and self.lane_width:
            rospy.loginfo(f"Lane Length: {self.lane_length:.2f} m, Lane Width: {self.lane_width:.2f} m")

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
        # Collect lane lengths before moving straight
        self.collect_lane_lengths()
        self.collect_lane_widths()
        rospy.sleep(2.0)
        if self.lane_length is not None:
            self.straight_line(self.lane_length - 0.4)
        else:
            rospy.logwarn("Lane length not detected yet.")
        # self.straight_line(1.3)
        rospy.sleep(2.0)
        self.move_arc_quarter_odometry(0.2, 1.3, False)
        self.move_arc_quarter_odometry(0.2, 1.3, True)
        # self.turn_90_degrees()
        # rospy.sleep(2.0)
        # if self.lane_length is not None:
        #     self.straight_line(self.lane_length)
        # else:
        #     rospy.logwarn("Lane length not detected yet.")
        # rospy.sleep(2.0)
        # self.turn_90_degrees()
        # rospy.sleep(2.0)
        # if self.lane_length is not None:
        #     self.straight_line(self.lane_length)
        # else:
        #     rospy.logwarn("Lane length not detected yet.")
        # rospy.sleep(2.0)
        # self.turn_90_degrees()
        # rospy.sleep(2.0)
        # if self.lane_length is not None:
        #     self.straight_line(self.lane_length)
        # else:
        #     rospy.logwarn("Lane length not detected yet.")
        # rospy.sleep(2.0)
        # self.turn_90_degrees()
        
        
if __name__ == "__main__":
    node = DShapeNode(node_name="d_shape_node")
    node.run()
    rospy.loginfo("DShapeNode main finished.")

