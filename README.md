# Duckiebot ROS Assignment

This repository contains the code and documentation for the ROS-based Duckiebot assignment. We familiarized ourselves with ROS and Duckiebot functionalities and implemented various functions including odometry using wheel encoders, rotating the robot, and making the Duckiebot follow a specific path while utilizing ROS services and visual feedback via LED lights. Furthermore, we also implement the lane-tracing functions and image recognition functionalities.

---

# Part One - Computer Vision

Understand camera intrinsic parameters and their role in image formation. We will learn how to correct distorted images using intrinsic parameters to obtain undistorted views. Then, we will implement color detection techniques to detect different lane colors and perform different behaviors corresponding to each color.

---

## 1. Camera Distortion

- **a.** Subscribed to the camera topic to get the distorted image.  
- **b.** Used our camera intrinsic calibration parameters to manually convert (transform) the distorted image to become undistorted.  
- **c.** Created a publisher for undistorted images.
- ###Distorted_Camera.py

---

## 2. Image Pre-Processing

- **a.** Resized the image.
- **b.** Applied image smoothing (blurring) using OpenCV (both tasks are done in one file).
- ###Resized_Blurred.py

---

## 3. Color Detection

- **a.** Used the bot’s camera to display the 3 lines, and screenshoted 3 images, where each image shows a blue, red, and green line. We do that in Colour_Detection.py
- **b.** From the saved images, we determined the lower and upper HSV values for each lane color using appropriate methods. You can do that in file called hsv_values.py (without using a .sh). 
- **c.** Implemented image contouring and color detection using OpenCV. Once again can be done without the '.sh' file and just ran from image_contouring.py
- **d.** Draw a rectangle surrounding the detected lane. That's done in file called detect.py
- **e.** Obtain lane dimensions with respect to the bot’s camera. That's done in file called detect.py as well.

---

## 4. LED Controller

- **a.** We re-used led-control code from lab 2. 

---

## 5. Autonomous Navigation Functions

- **a.** We re-used our shape following code from lab 2, however we commented out one of the turns in the run method. Should all be stored in D_Shape_Node.py

---

## 6. Lane-Based Behavioral Execution

- **a.** Combined color and line detection, LED controller, and navigation functions as requested:
  
  - **i. Blue Line:**
    1. Started 30 cm away from the blue line.
    2. Stoped before the line for 3 seconds.
    3. Signaled orange using the right side (front & back) LED.
    4. Moved in a curve through 90 degrees to the right.
    5. blue_movement.py
  
  - **ii. Red Line:**
    1. Started 30 cm away from the red line.
    2. Stoped before the line for 3 seconds.
    3. Moved straight for about 30 cm.
    4. red_movement.py
  
  - **iii. Green Line:**
    1. Started 30 cm away from the green line.
    2. Stoped before the line for 3 seconds.
    3. Signaled red using the left side (front & back) LED.
    4. Moved in a curve through 90 degrees to the left.
    5. green_movement.py


---

# Part Two - Autonomous Lane Following Controllers

This section explores in depth different types of controllers that enable the bot to drive autonomously.

---

## 1. Lane Detection Functions

- **a. Yellow Dotted Lane and White Solid Lane Detection:**  
  Implemented a function to detect the yellow dotted lane that separates inbound and outbound traffic, as well as a function to detect the white solid lane (the outer lane), all in one code. In that code we don't directly publish, however we do publish in all of the controller functions. 
  lane_detection_node.py

---

## 2. Controller Implementation for Lane Following

Implemented functions for the following controllers to perform lane following along a straight path for at least 1.5 meters.

- **a. Proportional (P) Controller:**  
  Developed a P controller to drive the bot along the lane based on a proportional error signal. Done in p_contr.py  

- **b. Proportional-Derivative (PD) Controller:**  
  Enhanced the controller by adding a derivative term to dampen oscillations and improve response during lane following. Done in pd_controller.py  

- **c. Proportional-Integral-Derivative (PID) Controller:**  
  Further refined the controller by including an integral term to eliminate steady-state errors, combining all three components for robust lane following. Done in pid_controller.py  

---

# Part Three - Full Lap Lane Following Integration

This part integrates computer vision and controllers to perform a full lap lane following using your code. The goal is to combine visual lane detection with controller logic to drive the bot autonomously along a full lap.

---

## 1. Lane Following Node and Basic Proportinal Controller

- **a. Create Lane Following Node:**
- We developed a ROS node and defined an error or target metric based on the lane position in p_controller.py
---

## 2. Proportional-Derivative (PD) Controller

- **a. Implementation:**
  We adjustsed the bot’s steering based on the calculated error from P and added the KD paramter to implement PD controller in pd_ctrl_follow.py
  Enhanced the basic P controller by adding a derivative term to form a PD controller.

---

## 3. Proportional-Integral-Derivative (PID) Controller

- **a. Implementation:**  
  Further extended the controller by adding an integral term to the PD controller to develop a full PID controller in pid_ctrl_follow.py.


---

# Conclusion

Throughout this assignment, we have gained comprehensive insights and practical experience in integrating computer vision with control strategies for autonomous navigation. The project was divided into three main parts:

- **Part One - Computer Vision:**  
  We explored the fundamentals of camera intrinsic parameters and image processing. By correcting camera distortion and implementing color detection, we learned how to extract meaningful information from visual data. This formed the basis for subsequent tasks where the bot relied on accurate image inputs for decision-making.

- **Part Two - Autonomous Lane Following Controllers:**  
  In this section, we developed functions to detect key lane markers (yellow dotted and white solid lanes) and published the detection results to ROS topics. We then implemented various controllers—Proportional (P), Proportional-Derivative (PD), and Proportional-Integral-Derivative (PID)—to perform lane following over a specified distance. This allowed us to analyze and compare the responsiveness and stability of each control strategy in real-world conditions. We struggled a bit with implementing lane-detection and figuring out the center of the lane to make the cotnrollers work initially. 

- **Part Three - Full Lap Lane Following Integration:**  
  The final part combined computer vision and controller techniques into a single node to execute a complete lap of autonomous lane following. Starting with a basic P controller, we evaluated its performance under different error conditions, then incrementally enhanced it with derivative and integral terms. This step-by-step integration showcased how each controller component contributes to improved error correction, stability, and overall performance in a dynamic environment. We struggled a bit with tuning the parameters in PD and PID controllers for lane following. 

Overall, the assignment provided valuable hands-on experience with ROS, computer vision techniques using OpenCV, and controller tuning for autonomous systems. We learned how to handle camera distortions, detect lane markers reliably, and fine-tune different controllers to achieve robust autonomous navigation.


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
dts devel run -R csc22911 -L <the_program_you_want_to_run>
```
For example, to run Color_Detection.py, execute:

```bash
dts devel run -R csc22911 -L col-det
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


NOTE: There is a folder called images, containing the 3 screenshots of the blue, red and green lines. This needs to be used for proper hsv_values.py and image_contouring.py function. 
