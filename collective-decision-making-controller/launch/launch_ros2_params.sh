#!/bin/bash

# The purpose of this script is to create a launch file for a multi-robot
# system by replicating a "group" tag "n" times, then executing the resulting
# launch file.

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).

n=20

LAUNCH_FILE=/tmp/argos_interface.launch.py

echo "import os" > $LAUNCH_FILE
echo -e "import pathlib" >> $LAUNCH_FILE
echo -e "import launch" >> $LAUNCH_FILE
echo -e "import yaml" >> $LAUNCH_FILE
echo -e "from launch import LaunchDescription" >> $LAUNCH_FILE
echo -e "from launch_ros.actions import Node" >> $LAUNCH_FILE

echo -e "from ament_index_python.packages import get_package_share_directory" >> $LAUNCH_FILE

echo -e "def generate_launch_description():" >> $LAUNCH_FILE
echo -e "\tconfig_dir = os.path.join('/home/sindiso/ros2_ws/src/collective-decision-making-controller', 'config')" >> $LAUNCH_FILE
echo -e "\tparam_config = os.path.join(config_dir, \"config.yaml\")" >> $LAUNCH_FILE
echo -e "\twith open(param_config, 'r') as f:" >> $LAUNCH_FILE
echo -e "\t\tparams = yaml.safe_load(f)[\"controller\"][\"ros__parameters\"]" >> $LAUNCH_FILE
    
echo -e "\tld = LaunchDescription()" >> $LAUNCH_FILE

for ((i=0; i<n; i++)); do
    namespace="bot$i"
    echo -e "\t$namespace = Node(package=\"collective_decision_making\", executable=\"controller\", name=\"controller\", output=\"screen\", namespace=\"$namespace\", parameters=[params])" >> $LAUNCH_FILE
    echo -e "\tld.add_action($namespace)" >> $LAUNCH_FILE
done >> $LAUNCH_FILE
echo -e "\treturn ld" >> $LAUNCH_FILE

ros2 launch $LAUNCH_FILE
