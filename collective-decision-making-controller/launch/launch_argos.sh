#!/bin/bash

# The purpose of this script is to set up the necessary environment variables for argos,
# anf then spin off the argos simulator.
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME//ros2_ws/install/loop_functions/lib
export ARGOS_PLUGIN_PATH=$HOME/ros2_ws/install/loop_functions/lib/

argos3 -c world.argos

