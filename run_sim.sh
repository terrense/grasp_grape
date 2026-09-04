#!/bin/bash
# helper: source everything and launch, logging to /tmp
source /opt/ros/noetic/setup.bash
source ~/grape_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=~/grape_ws/src/grape_harvest/models:/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_DATABASE_URI=
exec "$@"
