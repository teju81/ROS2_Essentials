# ROS2_Essentials


This repository will serve as a quick tutorial/recap and also template for your awesome robotics software projects that will be based on ROS2. I am using ROS2 Iron at the time of making this tutorial.


1. Create Workspace

Run the following commands to create your workspace
- mkdir -p ~/ros2_ws/src
- cd <ros2_ws>/src


2. Create a Package

(i) Your overall package structure should be like
  - <ros2_ws>/src
      - src 1
      - src 2
      .
      .
      .
      - src n

workspace_folder/  
    src/  
      cpp_package_1/  
          CMakeLists.txt  
          include/cpp_package_1/  
          package.xml  
          src/  
      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/

(ii) Each src folder is a package by itself. Each package has the following structure
  - package folder
    - include
    - src
    - package.xml
    - CMakeLists.txt


(iii) From the root folder <ros2_ws> run the following commands
- create pkg
- colcon build
- source install/setup.bash
- ros2 run <package_name> <node_executable_name>


3. Publish Subscribe to Topics
4. Services
5. Custom Messages
6. Action Servers
