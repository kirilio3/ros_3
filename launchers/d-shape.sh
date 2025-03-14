#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun my_package D_Shape_Node.py

# wait for app to end
dt-launchfile-join