#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float64 as Float
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, LEDPattern
from sensor_msgs.msg import CompressedImage
from D_shape_node import DShapeNode
from lane_detection_node import LaneDetectionNode

class DrawSquareNode(DShapeNode):
    
    def __init__(self, node_name):
        super(DrawSquareNode, self).__init__(node_name=node_name)
        self.pub_cmd = rospy.Publisher(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.led_pub = rospy.Publisher(f"/{self.vehicle_name}/led_emitter_node/led_pattern", LEDPattern, queue_size=1)
        self.sub_left_enc = rospy.Subscriber(f"/{self.vehicle_name}/left_wheel_encoder_node/tick", WheelEncoderStamped, self.cb_left_encoder)
        self.sub_right_enc = rospy.Subscriber(f"/{self.vehicle_name}/right_wheel_encoder_node/tick", WheelEncoderStamped, self.cb_right_encoder)
        self.LDNode = LaneDetectionNode(node_name='lane_detection_node')

        # data to be tuned
        self.TICKS_PER_REV = 135
        self.WHEEL_RADIUS = 0.0318
        self.WHEEL_CIRC = 2.0 * math.pi * self.WHEEL_RADIUS
        self.BASELINE = 0.1016
        self.ROTATE_90_RAD = math.pi / 2
        self.TOL_ANGLE = 0
        self.TOL = 0.08
        self.OMEGA_SPEED = 10
        self.VELOCITY = 0.4
        self.small_tune = 0
        self._left_distance_traveled = 0.0
        self._right_distance_traveled = 0.0
        self.TICKS_PER_REV = 135
        self.WHEEL_RADIUS = 0.0318

    def run(self):
        for _ in range(4):
            self.lane_detection()
            self.straight_line(distance=1)
            rospy.sleep(1)
            self.rotate_to_angle(self.ROTATE_90_RAD, direction=-1)
            rospy.sleep(1)


    def lane_detection(self):
        rospy.loginfo(f"height:{self.LDNode.height_in_meters}")

if __name__ == '__main__':
    # create the node
    node = DrawSquareNode(node_name='draw_square_node')
    # run node
    node.run()
    # keep spinning
    # rospy.spin()