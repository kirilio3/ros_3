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
from sensor_msgs.msg import CompressedImage
import subprocess
import atexit

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

        # Subscriber for camera image
        self._camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self.sub_camera = rospy.Subscriber(self._camera_topic, CompressedImage, self.cb_camera)
        self._bridge = CvBridge()

        # Store the last tick count (None until we get first reading)
        self.last_left_ticks  = None
        self.last_right_ticks = None

        # ---- TUNABLE PARAMETERS ----
        self.TICKS_PER_REV = 135            # typical for Duckietown wheel encoder
        self.WHEEL_RADIUS  = 0.0318         # ~3.18 cm radius
        self.WHEEL_CIRC    = 2.0 * math.pi * self.WHEEL_RADIUS  # circumference in meters
        self.BASELINE      = 0.077            # distance between wheels in meters (approx)

        self.TOL = 0.08     
        # self.TOL_ANGLE = 0.174533          # 10 degrees in radians
        self.TOL_ANGLE = 0

        # Desired angles (radians)
        self.three_sixty = math.pi                  # 360 degrees
        self.one_eighty = self.three_sixty / 2      # 180 degrees
        self.ninety = self.three_sixty / 4          # 90 degrees

        # Angular speed (rad/s) for in-place rotation
        self.OMEGA_SPEED = 10  # tune me
        self.angular_vel = 2.6

        self.VELOCITY = 0.4  # m/s
        self.VELOCITY_LEFT = 0.42  # Adjust for left wheel if needed
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
        signal.signal(signal.SIGTERM, self.signal_handler)

        # Register cleanup function
        atexit.register(self.cleanup)

        # Green line detection flag
        self.green_line_reached = False

        # List to store child processes
        self.child_processes = []

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

    def cb_camera(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        processed_image, green_distance = self.detect_green_line(image)
        
        if green_distance is None:
            self.green_line_reached = True
            rospy.loginfo("Green line reached, stopping the robot.")
            return
            # self.stop()

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
                focal_length = 314.9
                # focal_length = 50  # Approximate focal length
                real_height_meters = 0.1  # Estimated real-world height of the green line (adjust if necessary)
                pixel_height = h
                
                if pixel_height > 0:
                    distance = abs((real_height_meters * focal_length) / (pixel_height - (y//2)))
                    # distance = (real_height_meters * focal_length) / pixel_height
                    
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    cv2.putText(image, f"Green Line Distance: {distance:.2f} m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    return image, distance
        
            else:
                # If the line is out of view, assume it's reached
                cv2.putText(image, "Line reached", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                self.green_line_reached = True
                return image, None
        
        # If no green line is detected
        return image, None
    
    def straight_line(self, distance):
        rospy.loginfo(f"Moving forward {distance} meters...")
        
        forward_msg = Twist2DStamped(v=self.VELOCITY, omega=0.0)
        self.set_led_color('GREEN')
        self.pub_cmd.publish(forward_msg)
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown() and not self.green_line_reached:
            heading_error = self.compute_heading()
            if abs(heading_error) > 0.05:
                correction = 0.1 * heading_error
                forward_msg = Twist2DStamped(v=self.VELOCITY, omega=-correction)
            self.pub_cmd.publish(forward_msg)

            # Stop if the distance is reached
            if (self._right_distance_traveled + self._left_distance_traveled) / 2 >= distance:
                self.stop()
                break
            rate.sleep()
        
        self.stop()  # Ensure stopping even if the loop exits

    def move_arc_quarter_odometry(self, radius, speed, clockwise=False):

        # 1) Reset odometry, measure initial heading
        rospy.sleep(0.01)  # let odometry callbacks update

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

        rate = rospy.Rate(200)
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
        
        # Define the default color for all LEDs (e.g., GREEN or PURPLE)
        default_color = ColorRGBA(0, 1, 0, 1)  # Green as default
        
        # Define the OFF color (BLACK)
        off_color = ColorRGBA(0, 0, 0, 0)  # Black (OFF)

        # Initialize the color list and RGB values for all 5 LEDs
        pattern.color_list = ["GREEN"] * 5  # Default color for all LEDs
        pattern.rgb_vals = [default_color] * 5  # Default RGB values for all LEDs
        
        # Set the color for the left front and back LEDs (indices 0 and 3)
        if color == 'RED':
            selected_color = ColorRGBA(1, 0, 0, 1)  # Red color
            pattern.color_list[0] = "RED"  # Left front LED
            pattern.color_list[4] = "RED"  # Left back LED
            pattern.rgb_vals[0] = selected_color  # Left front LED
            pattern.rgb_vals[4] = selected_color  # Left back LED

            # Turn off the other LEDs (set to BLACK)
            pattern.color_list[1] = "OFF"  # Right front LED
            pattern.color_list[2] = "OFF"  # Front center LED
            pattern.color_list[3] = "OFF"  # Right back LED
            pattern.rgb_vals[1] = off_color  # Right front LED
            pattern.rgb_vals[2] = off_color  # Front center LED
            pattern.rgb_vals[3] = off_color  # Right back LED

        elif color == 'CYAN':
            selected_color = ColorRGBA(0.0, 1.0, 1.0, 1.0)  # Cyan color
            pattern.color_list = ["CYAN"] * 5  # Apply color to all LEDs
            pattern.rgb_vals = [selected_color] * 5  # Set the color
        elif color == 'GREEN':
            selected_color = ColorRGBA(0, 1, 0, 1)  # Green color
            pattern.color_list = ["GREEN"] * 5  # Apply color to all LEDs
            pattern.rgb_vals = [selected_color] * 5  # Set the color
        else:  # Default PURPLE
            selected_color = ColorRGBA(0.5, 0.0, 0.5, 1.0)  # Purple color
            pattern.color_list = ["PURPLE"] * 5  # Apply color to all LEDs
            pattern.rgb_vals = [selected_color] * 5  # Set the color
        
        # Set the color mask to affect all LEDs
        pattern.color_mask = [1, 1, 1, 1, 1]  # Affect all LEDs
        pattern.frequency = 1.0  # Optional: Blinking speed
        pattern.frequency_mask = [1, 1, 1, 1, 1]  # Apply frequency to all LEDs
        
        # Publish the LED pattern
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

    def cleanup(self):
        """Cleanup function to terminate child processes."""
        rospy.loginfo("Cleaning up...")
        for process in self.child_processes:
            process.terminate()
            process.wait()

    def compute_heading(self):
        return (self._right_distance_traveled - self._left_distance_traveled) / self.BASELINE

    def run(self):
        rospy.sleep(2.0)
        self.straight_line(1.3)
        rospy.sleep(3.0)
        self.move_arc_quarter_odometry(0.17, 1.18, False)
        
if __name__ == "__main__":
    node = DShapeNode(node_name="d_shape_node")
    node.run()
    rospy.loginfo("DShapeNode main finished.")
