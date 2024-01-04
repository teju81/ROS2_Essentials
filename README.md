# ROS2_Essentials


This repository will serve as a quick tutorial/recap and also template for your awesome robotics software projects that will be based on ROS2. I am using ROS2 Iron at the time of making this tutorial.


1. Create Workspace

Run the following commands to create your workspace
- mkdir <ros2_ws>
- cd <ros2_ws>
- mkdir src


2. Create a Package

(i) Your overall package structure will be like
  - <ros2_ws>/src
      - src 1
      - src 2
      .
      .
      .
      - src n

(ii) Each src folder is a package by itself. Each package has the following structure
  - package folder
    - include
    - src
    - package.xml
    - CMakeLists.txt


(iii) From the root folder <ros2_ws> run the following commands
- colcon build
- . <note the space> install/setup.bash
- ros2 run <package_name> <node_executable_name>


3. Publish Subscribe to Topics
4. Services
5. Custom Messages
6. Action Servers
