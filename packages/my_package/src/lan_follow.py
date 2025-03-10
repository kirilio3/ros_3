#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, LEDPattern

import math

class LaneFollower(DTROS):
    def __init__(self, node_name="lane_follower_node"):

        #########################################################################################################
        self.last_left_ticks  = None
        self.last_right_ticks = None

        # ---- TUNABLE PARAMETERS ----
        self.TICKS_PER_REV = 135            # typical for Duckietown wheel encoder
        self.WHEEL_RADIUS  = 0.0318         # ~3.18 cm radius
        self.WHEEL_CIRC    = 2.0 * math.pi * self.WHEEL_RADIUS  # circumference in meters
        self.BASELINE      = 0.1016            # distance between wheels in meters (approx)

        # Desired angles (radians)
        self.ROTATE_90_RAD = math.pi / 2    # 90 degrees
        # 0.174533          # 10 degrees in radians
        # 0.0872665          # 5 degrees in radians
        self.TOL_ANGLE = 0.174533         # 10 degrees in radians
        self.TOL = 0.08                   # A small tolerance

        # Angular speed (rad/s) for in-place rotation
        self.OMEGA_SPEED = 10  # tune me
        self.VELOCITY = 0.4  # m/s
        self.small_tune = 0    # not rotation, going straight

        # -----------------------------

        self._left_distance_traveled = 0.0
        self._right_distance_traveled = 0.0
        
        # Wheel/encoder parameters (these may vary for your Duckiebot)
        self.TICKS_PER_REV = 135    # typical for standard Duckietown wheel encoders
        self.WHEEL_RADIUS  = 0.0318 # meters (approx. radius for Duckiebot wheels)
        self.WHEEL_CIRCUM  = 2.0 * math.pi * self.WHEEL_RADIUS  # meters per revolution
        
        # Target distances
        self.FORWARD_DISTANCE  = 1.25
        self.BACKWARD_DISTANCE = -1.25
    #########################################################################################################




        # Initialize DTROS node (using a generic node type)
        super(LaneFollower, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.bridge = CvBridge()

        self.vehicle_name = os.environ['VEHICLE_NAME']
        self.left_encoder_topic  = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
        self.right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"

        # Publisher for velocity commands
        twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)


        self.sub_left_enc  = rospy.Subscriber(self.left_encoder_topic,  WheelEncoderStamped, self.cb_left_encoder)
        self.sub_right_enc = rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.cb_right_encoder)

        # Subscribers and publishers
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.yellow_pub = rospy.Publisher("/lane_detection/yellow", Image, queue_size=10)
        self.white_pub  = rospy.Publisher("/lane_detection/white", Image, queue_size=10)

        # Controller gains for P, PD, and PID controllers
        self.kp = rospy.get_param("~kp", 0.005)
        self.kd = rospy.get_param("~kd", 0.001)
        self.ki = rospy.get_param("~ki", 0.0005)

        # Variables for derivative and integral calculations
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.Time.now()

        # Controller selection: "P", "PD", or "PID"
        self.controller_type = rospy.get_param("~controller_type", "P")

    def image_callback(self, img_msg):
        try:
            # Convert the ROS image to an OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return

        # Process the image for lane detection
        yellow_img, yellow_error = self.detect_yellow_lane(cv_image.copy())
        white_img, white_error   = self.detect_white_lane(cv_image.copy())

        # Publish the annotated images to their respective topics
        try:
            self.yellow_pub.publish(self.bridge.cv2_to_imgmsg(yellow_img, "bgr8"))
            self.white_pub.publish(self.bridge.cv2_to_imgmsg(white_img, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

        # For lane following, choose one lane to follow.
        # Here we follow the white lane if it is detected.
        if white_error is not None:
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time

            # Compute the steering command based on the chosen controller
            if self.controller_type == "P":
                steer = self.p_controller(white_error)
            elif self.controller_type == "PD":
                steer = self.pd_controller(white_error, dt)
            elif self.controller_type == "PID":
                steer = self.pid_controller(white_error, dt)
            else:
                steer = 0.0

            # Create and publish a Twist message with a constant forward velocity.
            twist = Twist2DStamped(v=self.VELOCITY, omega=steer)
            self.pub_cmd.publish(twist)
        else:
            # Stop the robot if no lane is detected.
            twist = Twist2DStamped(v=self.VELOCITY, omega=steer)
            self.pub_cmd.publish(twist)


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

    def detect_yellow_lane(self, image):
        """
        Detect the yellow dotted lane (divider between inbound and outbound traffic)
        and compute the horizontal error (difference between lane center and image center).
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Clean the mask with morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
        mask_yellow = cv2.dilate(mask_yellow, kernel, iterations=2)

        contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lane_error = None
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 255), 2)
            cv2.putText(image, "Yellow Lane", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            lane_center = x + w / 2.0
            image_center = image.shape[1] / 2.0
            lane_error = lane_center - image_center
        return image, lane_error

    def detect_white_lane(self, image):
        """
        Detect the white solid lane (outer lane) and compute the horizontal error
        (difference between lane center and image center).
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((5, 5), np.uint8)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
        mask_white = cv2.dilate(mask_white, kernel, iterations=2)

        contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lane_error = None
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(image, (x, y), (x+w, y+h), (255, 255, 255), 2)
            cv2.putText(image, "White Lane", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            lane_center = x + w / 2.0
            image_center = image.shape[1] / 2.0
            lane_error = lane_center - image_center
        return image, lane_error

    # ---------- Controller Functions ----------

    def p_controller(self, error):
        """
        Proportional (P) Controller:
          steering = kp * error
        """
        return self.kp * error

    def pd_controller(self, error, dt):
        """
        Proportional-Derivative (PD) Controller:
          steering = kp * error + kd * (error - previous_error)/dt
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


    def straight_line(self, distance):
        rospy.loginfo(f"Moving forward {distance} meters...")
        forward_msg = Twist2DStamped(v=self.VELOCITY, omega=0)
        self.set_led_color('GREEN')  # Set LED color to GREEN when moving straight
        self.pub_cmd.publish(forward_msg)
        rate = rospy.Rate(10)
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
    # ---------- Run Function ----------
    def stop(self):
        msg = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd.publish(msg)

    def on_shutdown(self):
        self.stop()
        super(LaneFollower, self).on_shutdown()

    def run(self):
        """
        Runs the lane following task. The robot will drive forward along the lane for
        approximately 1.5 meters (given a constant forward velocity of 0.2 m/s, that takes ~7.5 seconds).
        After that, the robot stops and the node shuts down.
        """
        rospy.loginfo("Starting lane following task for 1.5 meters...")
        rospy.sleep(1)  # Wait for the subscribers to connect
        self.straight_line(1.5)  # Move forward 1.5 meters
        rospy.sleep(1)  # Wait for the robot to stop before shutting down the node

if __name__ == '__main__':
    try:
        lane_follower = LaneFollower()
        rospy.spin()
        
        # lane_follower.run()  # Start the lane following task and run the node
    except rospy.ROSInterruptException:
        pass
