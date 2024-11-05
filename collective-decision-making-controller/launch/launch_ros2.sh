#!/bin/bash

# The purpose of this script is to create a launch file for a multi-robot
# system by replicating a "group" tag "n" times, then executing the resulting
# launch file.

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).

n=20

LAUNCH_FILE=/tmp/argos_interface.launch.xml

echo "<launch>" > $LAUNCH_FILE

for ((i=0; i<n; i++)); do
    namespace="bot$i"
    echo -e "\t<group>"
    echo -e "\t\t<node pkg=\"collective_decision_making\" exec=\"controller\" name=\"controller\" output=\"screen\" namespace=\"$namespace\" args=\"$namespace\"/>"
    echo -e "\t</group>"
done >> $LAUNCH_FILE
echo -e "</launch>" >> $LAUNCH_FILE

ros2 launch $LAUNCH_FILE
