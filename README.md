# ROS2 101


This repository will serve as a quick tutorial/recap and also template for your awesome robotics software projects that will be based on ROS2. I am using ROS2 Iron on Ubuntu 22.04 at the time of making this tutorial.


## 1 Create Workspace

Run the following commands to create your workspace
```
mkdir -p ~/ros2_ws/src
cd <ros2_ws>
colcon build
```

And you will see that colcon has created new directories:

```
build  install  log  src
```
The install directory is where your workspace’s setup files are, which you can use to source your overlay.


Open a new terminal and run the following command
```
source install/setup.bash
```
it is very important that you open a new terminal, separate from the one where you built the workspace. Sourcing an overlay in the same terminal where you built, or likewise building where an overlay is sourced, may create complex issues.

## 2 Packages

### 2.1 Creating a Package

Make sure you are in the src folder before running the package creation command

```
cd ~/ros2_ws/src
```

The command syntax for creating a new CPP package in ROS 2 is:

```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>
```

The command syntax for creating a new python package in ROS 2 is:

```
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
```

You will now have a new folder within your workspace’s ``src`` directory called ``my_package``.


### 2.2 Structure of a Package

Packages can be written in C++ or Python. For setup, each type requires some boiler plate code and structure to be followed.

#### 2.2.1 C++ Package

The simplest possible CPP package may have a file structure that looks like:

```
my_package/
     CMakeLists.txt
     include/my_package/
     package.xml
     src/
```
The ``src`` folder will contain the source code that implements ROS nodes such as a publishers, subscribers, custom messages, services, action servers etc (more on these implementation details later). The ``include`` folder will contain any header files that is required. In addition to the code, two files ``package.xml`` and ``CMakeLists.txt`` need to be setup properly to be able to run the ROS nodes in the package.

**package.xml**

The barebones ``package.xml`` will look like this
```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <!-- Package name must match the project name given in CMakeLists.txt -->
  <name>multi_robot_cpp_publisher_package</name>
  <version>1.0.0</version>
  <description>Package for Multi Robot CPP Publishers</description>
  <maintainer email="teju81@gmail.com">Raviteja U.</maintainer>
  <license>Apache-2.0</license>
  <author email="teju81@gmail.com">Raviteja U.</author>

  <buildtool_depend>ament_cmake</buildtool_depend>


  <!-- Build dependencies and executable dependencies will be added here. This is package specific. More info about what to put in this section can be found later in this tutorial.-->

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
**CMakeLists.txt**

The barebones ``CMakeLists.txt`` will look like this
```
cmake_minimum_required(VERSION 3.8)
# Project name given must match the package name in package.xml
project(multi_robot_cpp_publisher_package)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Dependencies: More package specific dependencies can be added below the next line
find_package(ament_cmake REQUIRED)


# Add Executables: More package specific executables can be added below. More info on this section will come later in this tutorial.


install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

```

#### 2.2.2 Python Package

The simplest possible python package may have a file structure that looks like:

```
my_package/
      package.xml
      resource/my_package
      setup.cfg
      setup.py
      my_package/
```

A single workspace can contain as many packages as you want, each in their own folder.
You can also have packages of different build types in one workspace (CMake, Python, etc.).
You cannot have nested packages.

Best practice is to have a ``src`` folder within your workspace, and to create your packages in there.
This keeps the top level of the workspace “clean”.

A trivial workspace might look like:

```
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
```


## 3 Publishers and Subscribers

Nodes are executable processes that communicate over the ROS graph. In this tutorial, the nodes will pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

### 3.1 Writing a simple publisher and subscriber in C++

**Publisher**

Below is the code for a minimal publisher.

```
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MultiRobotCppPublisherClass : public rclcpp::Node
{
public:
  MultiRobotCppPublisherClass()
  : Node("multi_robot_cpp_publisher_node"), count_(0), queue_size_(10)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("multi_robot_topic", queue_size_);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MultiRobotCppPublisherClass::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  size_t queue_size_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiRobotCppPublisherClass>());
  rclcpp::shutdown();
  return 0;
}
```

**Subscriber**

Below is the code for a minimal subscriber.

```
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MultiRobotCppSubscriber : public rclcpp::Node
{
  public:
    MultiRobotCppSubscriber()
    : Node("multi_robot_subscriber_node"), queue_size_(10)
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "multi_robot_topic", queue_size_, std::bind(&MultiRobotCppSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t queue_size_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiRobotCppSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

### 3.2 Writing a simple publisher and subscriber in Python

**Publisher**
This is a simple python publisher.

```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MultiRobotPyPublisherClass(Node):

    def __init__(self):
        super().__init__('multi_robot_py_publisher_node')
        self.queue_size_ = 10
        self.publisher_ = self.create_publisher(String, 'multi_robot_topic', self.queue_size_)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    multi_robot_py_publisher_obj = MultiRobotPyPublisherClass()

    rclpy.spin(multi_robot_py_publisher_obj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multi_robot_py_publisher_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Subscriber**

This is a simple subscriber.

```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MultiRobotPySubscriberClass(Node):

    def __init__(self):
        super().__init__('multi_robot_subscriber_node')
        self.queue_size_ = 10
        self.subscription = self.create_subscription(
            String,
            'multi_robot_topic',
            self.listener_callback,
            self.queue_size_)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    multi_robot_py_subscriber_obj = MultiRobotPySubscriberClass()

    rclpy.spin(multi_robot_py_subscriber_obj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multi_robot_py_subscriber_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```



4. Services
5. Custom Messages
6. Action Servers
