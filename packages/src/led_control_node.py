#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool, SetBoolResponse  # Simple service for toggling LED states

# Define colors
BLUE = ColorRGBA(0, 0, 1, 1)   # Blue
GREEN = ColorRGBA(0, 1, 0, 1)  # Green
CYAN = ColorRGBA(0.0, 1.0, 1.0, 1.0)
PURPLE = ColorRGBA(0.5, 0.0, 0.5, 1.0)

class LEDControlNode(DTROS):
    def __init__(self, node_name):
        super(LEDControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        vehicle_name = os.environ['VEHICLE_NAME']
        self.led_topic = f"/{vehicle_name}/led_emitter_node/led_pattern"

        # Publisher for LED control
        self.led_pub = rospy.Publisher(self.led_topic, LEDPattern, queue_size=1)

        # ROS Service to change LED state
        self.srv = rospy.Service('set_led_state', SetBool, self.handle_led_request)

    def handle_led_request(self, req):
        """ Callback for the service, toggles LED state on all LEDs """
        
        # Define colors for both states
        selected_color = CYAN if req.data else PURPLE

        pattern = LEDPattern()
        pattern.color_list = ["CYAN" if req.data else "PURPLE"] * 5  # 5 LEDs
        pattern.rgb_vals = [selected_color] * 5  # Apply color to all LEDs
        pattern.color_mask = [1, 1, 1, 1, 1]  # Affect all LEDs
        pattern.frequency = 1.0  # Optional: Blinking speed
        pattern.frequency_mask = [1, 1, 1, 1, 1]  # Apply frequency to all LEDs

        self.led_pub.publish(pattern)

        return SetBoolResponse(success=True, message=f"LEDs set to {'CYAN' if req.data else 'PURPLE'}")

    def run(self):
        # rospy.stop()
        rospy.spin()
    def on_shutdown(self):
        super(LEDControlNode, self).on_shutdown()

if __name__ == '__main__':
    node = LEDControlNode(node_name="led_control_node")
    node.run()





# #!/usr/bin/env python3

# import os
# import rospy
# from duckietown.dtros import DTROS, NodeType
# from duckietown_msgs.msg import LEDPattern

# # Define LED colors for different states
# STATE_1_COLORS = ["blue"] * 5  # 5 LEDs set to blue
# STATE_2_COLORS = ["green"] * 5  # 5 LEDs set to green

# class LEDControlNode(DTROS):
#     def __init__(self, node_name):
#         # Initialize DTROS parent class
#         super(LEDControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

#         # Get vehicle name from environment
#         vehicle_name = os.environ['VEHICLE_NAME']
#         self.led_topic = f"/{vehicle_name}/led_emitter_node/led_pattern"

#         # Construct publisher
#         self._publisher = rospy.Publisher(self.led_topic, LEDPattern, queue_size=1)

#         # Initialize state
#         self.current_state = 1

#     def run(self):
#         rate = rospy.Rate(1)  # 1 Hz blinking

#         while not rospy.is_shutdown():
#             pattern = LEDPattern()
#             pattern.color_list = STATE_1_COLORS if self.current_state == 1 else STATE_2_COLORS
#             pattern.frequency = 2.0  # Blinking frequency

#             # Toggle between states
#             self.current_state = 2 if self.current_state == 1 else 1

#             rospy.loginfo(f"Publishing LED pattern: {pattern.color_list}")
#             self._publisher.publish(pattern)
#             rate.sleep()

#     def on_shutdown(self):
#         rospy.loginfo("Shutting down: Turning off LEDs.")
#         pattern = LEDPattern()
#         pattern.color_list = ["off"] * 5  # Turn off all LEDs
#         self._publisher.publish(pattern)

# if __name__ == '__main__':
#     # Create the node
#     node = LEDControlNode(node_name='led_control_node')
#     # Run node
#     node.run()
#     # Keep the process from terminating
#     rospy.spin()




# #!/usr/bin/env python3

# import os
# import rospy
# from duckietown.dtros import DTROS, NodeType
# from std_srvs.srv import Trigger, TriggerResponse
# from duckietown_msgs.msg import LEDPattern

# # Define states for different colors
# STATE_BLUE = "blue"
# STATE_GREEN = "green"
# STATE_RED = "red"

# class LEDControlNode(DTROS):
#     def __init__(self, node_name):
#         super(LEDControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
#         # Get vehicle name from environment
#         vehicle_name = os.environ['VEHICLE_NAME']
        
#         # LED control topic
#         led_topic = f"/{vehicle_name}/led_emitter_node/led_pattern"
        
#         # Create publisher for LED pattern
#         self._publisher = rospy.Publisher(led_topic, LEDPattern, queue_size=1)
        
#         # Initialize state variable
#         self.state = STATE_BLUE  # Default state
        
#         # Initialize the service for changing the LED color
#         self._service = rospy.Service('change_led_color', Trigger, self.change_led_color)

#     def change_led_color(self, req):
#         """
#         Service callback to change the LED color
#         """
#         if self.state == STATE_BLUE:
#             self.state = STATE_GREEN
#         elif self.state == STATE_GREEN:
#             self.state = STATE_RED
#         else:
#             self.state = STATE_BLUE

#         # Publish the new LED color
#         self.publish_led_color(self.state)
        
#         return TriggerResponse(success=True, message=f"LED color changed to {self.state}")

#     def publish_led_color(self, color):
#         """
#         Publish the color to the LED pattern topic
#         """
#         led_msg = LEDPattern()
        
#         # Set LED pattern based on the color
#         if color == STATE_BLUE:
#             led_msg.pattern = [0, 0, 255]  # Blue
#         elif color == STATE_GREEN:
#             led_msg.pattern = [0, 255, 0]  # Green
#         elif color == STATE_RED:
#             led_msg.pattern = [255, 0, 0]  # Red

#         # Publish the color to the LED pattern topic
#         self._publisher.publish(led_msg)

#     def run(self):
#         # Keep the node running
#         rospy.spin()

# if __name__ == '__main__':
#     # Initialize ROS node
#     node = LEDControlNode(node_name='led_control_node')
    
#     # Run the node
#     node.run()
