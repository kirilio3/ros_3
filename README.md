Duckiebot ROS Assignment
This repository contains the code and documentation for the ROS-based Duckiebot assignment. We familiarized ourself with ROS and duckiebot functionalities and implemented various functions including odometry using wheel encoders, rotating the robot, and making the Duckiebot follow a specific path while utilizing ROS services and visual feedback via LED lights.

Part I: Getting Comfortable with ROS
1. ROS Wiki Concepts:
Nodes, Topics, Services, Messages, and Bags were explored and explained in detail. Communication setup between nodes was established and tested.

2. Using ROS with Duckiebots:
We set up DTROS and created a catkin package. Implemented the first ROS publisher and subscriber to communicate between nodes.

3. Basic Operations with Camera:
Created a subscriber for the camera feed. Processed the image: converted to grayscale, annotated with robot's hostname and image size. camera_processing_node.py Published the annotated image on a custom topic. Used rqt_image_view to visualize the image and included a screenshot.

Part II: Odometry Using Wheel Encoders
1. Wheel Encoder Data:
Subscribed to data from wheel encoders and published it for control.

2. Straight Line Task:
Successfully moved the Duckiebot forward and backward by 1.25 meters, measured deviations from desired location. straight_line_task.py Answered questions about speed and deviations.

3. Rotation Task:
Rotated the Duckiebot 90 degrees clockwise and then back to 0 degrees counterclockwise. rotate_node.py Observed and analyzed deviations, discussed possible causes.

4. Bag File Creation:
Saved the odometry data from both straight line and rotation task to a csc22911_rotate_odometry.bag and csc22911_straight_line_odometry.bag.

Loaded the rosbag in a Python script compute_trajectory.py for straight line and compute_trajectory_rotate.py, then plotted individual trajectories to visualize the movement of the Duckiebot. They both can be run from VSCode.

Part III: Playing with Duckiebots
1. LED Light Feedback:
Implemented a ROS service in led_control_node.py to change the LED light color based on the Duckiebot's state (e.g., blue for one state, green for another). Documented the state-to-color mapping. You can run it to test through rosservice call /set_led_state "data: true", rosservice call /set_led_state "data: false". However, the code was utilized properly in ourn D-shape follow execution.

2. Multiple ROS Nodes:
Implemented two ROS nodes: one for task execution and one for controlling the LED lights D_Shape_Node.py and led_control_node.py. Hopefully our termination of nodes after task completion is proper.

3. Path Following:
Kept the Duckiebot stationary for 5 seconds with the LED in one color. Moved the Duckiebot along a D-shaped path: a straight segment and a semi-circular segment, with the LED light color updated. Couldn't however get the robot to ideally return to the start position.

We didn't have enough time to complete tasks 4 and 5 of part III. Our robot died.
Conclusion
By completing this assignment, we gained hands-on experience with ROS communication, task automation, and data analysis using ROS bags.
`dt-launcher-my-launcher`.

When launching a new container, you can simply provide `dt-launcher-my-launcher` as
command.
