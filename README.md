## 🧩 Overview

This package bridges the ARGoS simulator with the ROS 2 ecosystem, enabling seamless integration of robot simulation with standard ROS tools and workflows.

### ✨ Key Features

- **ROS 2 Topic Compatibility**  
  All communication strictly follows ROS 2 topic conventions. Users do not need to worry about message formatting or conversions.

- **Plug-and-Play Simulation Bridge**  
  Automatically connects ARGoS robot sensors and actuators to corresponding ROS 2 topics, with no additional setup required.

- **Supports Time Synchronization and Simulation Acceleration**  
   Supports both real-time and simulation-time modes. Users can freely choose the time source (`/clock` or system time), enabling precise control for time-sensitive applications and accelerated simulation under virtual time.

This bridge provides a clean and unified interface for simulating large-scale, distributed multi-robot systems, making it ideal for ROS 2-based algorithm development and system testing.
