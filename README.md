# Duckiebot ROS Assignment

This repository contains the code and documentation for the ROS-based Duckiebot assignment. We familiarized ourselves with ROS and Duckiebot functionalities and implemented various functions including odometry using wheel encoders, rotating the robot, and making the Duckiebot follow a specific path while utilizing ROS services and visual feedback via LED lights. Furthermore, we also implement the lane-tracing functions and image recognition functionalities.

---

# Part One - Computer Vision

Understand camera intrinsic parameters and their role in image formation. We will learn how to correct distorted images using intrinsic parameters to obtain undistorted views. Then, we will implement color detection techniques to detect different lane colors and perform different behaviors corresponding to each color.

---

## 1. Camera Distortion

- **a.** Subscribe to the camera topic to get the distorted image.  
- **b.** Use your camera intrinsic calibration parameters to manually convert (transform) the distorted image to become undistorted.  
- **c.** Create a publisher for undistorted images.

---

## 2. Image Pre-Processing

- **a.** Resize the image.
- **b.** Apply image smoothing (blurring) using OpenCV.

---

## 3. Color Detection

- **a.** Using the bot’s camera, save 3 images locally, where each image shows a blue, red, and green line.
- **b.** From the saved images, determine the lower and upper HSV values for each lane color using appropriate methods.
- **c.** Implement image contouring and color detection using OpenCV *(reference: GeeksForGeeks)*.
- **d.** Draw a rectangle surrounding the detected lane.
- **e.** Obtain lane dimensions with respect to the bot’s camera.

---

## 4. LED Controller

- **a.** Develop a function to control the robot’s front and back LEDs, allowing the desired color to be passed as a string or an RGB value.  

---

## 5. Autonomous Navigation Functions

- **a.** Implement movement functions *(reuse functions from Exercise 2 if needed)*:
  - **i.** Move in a straight line for a specified distance.  
  - **ii.** Move in a curve through 90 degrees to the right.
  - **iii.** Move in a curve through 90 degrees to the left.
  - **iv.** Stop the bot for a specified duration.  
- **b.** Combine LED control with the moving functions.

---

## 6. Lane-Based Behavioral Execution

- **a.** Combine color and line detection, LED controller, and navigation functions to perform the following behaviors:
  
  - **i. Blue Line:**
    1. Start at least 30 cm away from the blue line.
    2. Stop before the line for 3-5 seconds.
    3. Signal using the right side (front & back) LED.
    4. Move in a curve through 90 degrees to the right.
  
  - **ii. Red Line:**
    1. Start at least 30 cm away from the red line.
    2. Stop before the line for 3-5 seconds.
    3. Move straight for at least 30 cm.
  
  - **iii. Green Line:**
    1. Start at least 30 cm away from the green line.
    2. Stop before the line for 3-5 seconds.
    3. Signal using the left side (front & back) LED.
    4. Move in a curve through 90 degrees to the left.


---

# Part Two - Autonomous Lane Following Controllers

This section explores in depth different types of controllers that enable the bot to drive autonomously.

---

## 1. Lane Detection Functions

- **a. Yellow Dotted Lane Detection:**  
  Implement a function to detect the yellow dotted lane that separates inbound and outbound traffic.
  
- **b. White Solid Lane Detection:**  
  Implement a function to detect the white solid lane (the outer lane).

- **c. Publish Detection Results:**  
  Publish the detection results for both the yellow and white lanes to a ROS topic.

---

## 2. Controller Implementation for Lane Following

Implement functions for the following controllers to perform lane following along a straight path for at least 1.5 meters. Use the useful resources provided as guidance.

- **a. Proportional (P) Controller:**  
  Develop a P controller to drive the bot along the lane based on a proportional error signal.  

- **b. Proportional-Derivative (PD) Controller:**  
  Enhance the controller by adding a derivative term to dampen oscillations and improve response during lane following.

- **c. Proportional-Integral-Derivative (PID) Controller:**  
  Further refine the controller by including an integral term to eliminate steady-state errors, combining all three components for robust lane following.

---

# Part Three - Full Lap Lane Following Integration

This part integrates computer vision and controllers to perform a full lap lane following using your code. The goal is to combine visual lane detection with controller logic to drive the bot autonomously along a full lap.

---

## 1. Lane Following Node

- **a. Create Lane Following Node:**  
  Develop a ROS node that performs lane following.  

- **b. Error/Target Calculation:**  
  Define an error or target metric based on the lane position. This error will be minimized by the controller to keep the bot on track.

---

## 2. Basic Proportional Controller

- **a. Implementation:**  
  Start with a basic Proportional (P) controller that adjusts the bot’s steering based on the calculated error.


---

## 3. Proportional-Derivative (PD) Controller

- **a. Implementation:**  
  Enhance the basic P controller by adding a derivative term to form a PD controller.

---

## 4. Proportional-Integral-Derivative (PID) Controller

- **a. Implementation:**  
  Further extend the controller by adding an integral term to the PD controller to develop a full PID controller.


---

This section details the integration of computer vision with controller strategies to achieve robust and reliable autonomous lane following. Experiment with each controller type, analyze performance, and document your observations regarding responsiveness and error correction.

# Conclusion

Throughout this assignment, we have gained comprehensive insights and practical experience in integrating computer vision with control strategies for autonomous navigation. The project was divided into three main parts:

- **Part One - Computer Vision:**  
  We explored the fundamentals of camera intrinsic parameters and image processing. By correcting camera distortion and implementing color detection, we learned how to extract meaningful information from visual data. This formed the basis for subsequent tasks where the bot relied on accurate image inputs for decision-making.

- **Part Two - Autonomous Lane Following Controllers:**  
  In this section, we developed functions to detect key lane markers (yellow dotted and white solid lanes) and published the detection results to ROS topics. We then implemented various controllers—Proportional (P), Proportional-Derivative (PD), and Proportional-Integral-Derivative (PID)—to perform lane following over a specified distance. This allowed us to analyze and compare the responsiveness and stability of each control strategy in real-world conditions.

- **Part Three - Full Lap Lane Following Integration:**  
  The final part combined computer vision and controller techniques into a single node to execute a complete lap of autonomous lane following. Starting with a basic P controller, we evaluated its performance under different error conditions, then incrementally enhanced it with derivative and integral terms. This step-by-step integration showcased how each controller component contributes to improved error correction, stability, and overall performance in a dynamic environment.

Overall, the assignment provided valuable hands-on experience with ROS, computer vision techniques using OpenCV, and controller tuning for autonomous systems. We learned how to handle camera distortions, detect lane markers reliably, and fine-tune different controllers to achieve robust autonomous navigation. These insights lay a strong foundation for further exploration in advanced robotics and autonomous driving applications.


# HOW TO RUN THE CODE
# Building and Running the Code

Follow the steps below to build and run the code:

---

## 1. Build the Code

First, build the code using the following command:

```bash
dts devel build -f
```
## 2. Run the Code
Next, run the desired program with:
```bash
dts devel run -R <duckiebot_name> -L <the_program_you_want_to_run>
```
For example, to run Color_Detection.py, execute:

```bash
dts devel run -R <duckiebot_name> -L col-det
```
This command will run the Color_Detection.py file.

The table shows the python file to its corresponding sh files
---
  
| Python File             | SH File               |
|-------------------------|-----------------------|
| Color_Detection.py      | col-det.sh            |
| D_Shape_Node.py         | d-shape.sh            |
| Distorted_Camera.py     | camera-distorted.sh   |
| Resized_Blurred.py      | res-bl.sh             |
| blue_movement.py        | blue.sh               |
| green_movement.py       | green.sh              |
| red_movement.py         | red.sh                |
| detect.py               | detect.sh             |
| lane_detection.py       | lane-detection.sh     |
| led_control_node.py     | led-control.sh        |
| p_controller.py         | p.sh                  |
| pd_controller.py        | pd-ctrl.sh            |
| pid_controller.py       | pid-ctrl.sh           |
| p_contr.py              | p-ctrl.sh             |
| pd_ctrl_follow.py       | pd-follow.sh          |
| pid_ctrl_follow.py      | pid-f.sh              |


NOTE: 
