#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float64 as Float
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, LEDPattern

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

        # Store the last tick count (None until we get first reading)
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

    def rotate_to_angle(self, target_angle, direction):
        rate = rospy.Rate(200)
        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.set_led_color('CYAN')  # Set LED color to CYAN when rotating
        while not rospy.is_shutdown():
            current_heading = self.compute_heading()
            rospy.loginfo(f"difference: {abs(current_heading - target_angle):.2f} rad")
            rospy.loginfo(f"    currnect heading: {current_heading:.2f} rad")
            rospy.loginfo(f"        right: {self._right_distance_traveled:.2f} rad")
            rospy.loginfo(f"        left: {self._left_distance_traveled:.2f} rad")  
            rospy.loginfo(f"    target angle: {target_angle:.2f} rad")
            rospy.loginfo(f"    cond1: {abs(current_heading - target_angle) < self.TOL}")
            rospy.loginfo(f"    cond2: {abs(current_heading) > abs(target_angle)}")
            if abs(current_heading - target_angle) < self.TOL or abs(current_heading) > abs(target_angle - self.TOL_ANGLE):
                # within tolerance => stop
                self._right_distance_traveled = 0.0
                self._left_distance_traveled = 0.0
                self.last_left_ticks = None
                self.last_right_ticks = None
                self.stop()
                break

            twist.omega = direction * self.OMEGA_SPEED
            self.pub_cmd.publish(twist)
            rate.sleep()

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

    def straight_line(self, distance):
        rospy.loginfo(f"Moving forward {distance} meters...")
        forward_msg = Twist2DStamped(v=self.VELOCITY, omega=self.small_tune)
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
        self.stop()
        super(DShapeNode, self).on_shutdown()

    def compute_heading(self):
        return (self._right_distance_traveled - self._left_distance_traveled) / self.BASELINE

    def run(self):
        rospy.sleep(2.0)
        self.straight_line(1.2)
        rospy.sleep(4.0)
        self.rotate_to_angle(-(self.ROTATE_90_RAD-self.TOL_ANGLE),-1.0)
        rospy.sleep(4.0)
        self.straight_line(1.2)
        rospy.sleep(4.0)
        self.rotate_to_angle(-(self.ROTATE_90_RAD-self.TOL_ANGLE),-1.0)
        rospy.sleep(4.0)
        self.straight_line(1.2)
        rospy.sleep(4.0)
        self.rotate_to_angle(-(self.ROTATE_90_RAD-self.TOL_ANGLE),-1.0)
        rospy.sleep(4.0)
        self.straight_line(1.2)
        rospy.sleep(4.0)
        self.rotate_to_angle(-(self.ROTATE_90_RAD-self.TOL_ANGLE),-1.0)

if __name__ == "__main__":
    node = DShapeNode(node_name="d_shape_node")
    node.run()
    rospy.loginfo("DShapeNode main finished.")