#!/bin/bash

source /environment.sh

dt-launchfile-init
rosrun my_package voice_control_node.py
dt-launchfile-join
