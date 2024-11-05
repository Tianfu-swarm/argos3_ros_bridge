# argos3_ros_bridge
argos3_ros_bridge is a package designed to integrate the ARGoS3 simulation environment with ROS (Robot Operating System). It provides a bridge that enables users to create and control robots in ARGoS3 while leveraging the powerful features and ecosystem of ROS. This package aims to offer a flexible and efficient solution for robotic research and development.

# What's ARGoS

ARGoS is a physics-based simulator designed to simulate large-scale robot swarms. Benchmark results show that ARGoS can perform physics-accurate simulation involving thousands of robots in a fraction of real time. ARGoS' main features are:

- Multi-threaded and deeply modular architecture, more flexible than any simulator with equivalent features.
- The possibility to run multiple physics engines at the same time.
- The possibility to divide the physical space into regions and assign different regions to different physics engines.

# Install
To install argos3_ros_bridge, use the following commands:
```bash
mkdir yours_ws/src
cd yours_ws/src
https://github.com/Tianfu-swarm/argos3_ros_bridge.git
cd ..
colcon build
```

# Usage
You can just load the generated controller into your bot.