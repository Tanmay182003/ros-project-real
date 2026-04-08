#!/bin/bash
# Voice control launcher - run from your laptop to control entebot208 by voice

docker run --rm \
  --name voice-control \
  --network host \
  --device /dev/snd \
  -v /run/user/1000/pulse:/run/user/1000/pulse \
  -e PULSE_SERVER=unix:/run/user/1000/pulse/native \
  -e VEHICLE_NAME=entebot208 \
  -e ROS_MASTER_URI=http://192.168.1.177:11311 \
  -e ROS_IP=$(hostname -I | awk '{print $1}') \
  --group-add audio \
  duckietown/ros-project-real:v3-amd64 \
  bash -c ". /environment.sh && . /opt/ros/noetic/setup.bash && . /code/devel/setup.bash && rosrun my_package voice_control_node.py"
