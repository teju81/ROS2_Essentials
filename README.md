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
</br>

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


<details>

<summary>Barebones package.xml File</summary>

```xml
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
</details>



<details>

<summary>Barebones CMakeLists.txt File</summary>

```
cmake_minimum_required(VERSION 3.8)
```
```cmake
# Project name given must match the package name in package.xml

project(multi_robot_cpp_publisher_package)
```
```
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
```
```cmake

# Add Executables: More package specific executables can be added below. More info on this section will come later in this tutorial.

install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)
```
```
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

</details></br>

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

<details>

<summary>Barebones package.xml File</summary>

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <!-- Package name must match the project name given in setup.py -->
  <name>multi_robot_py_publisher_package</name>
  <version>1.0.0</version>
  <description>Package for Multi Robot Py Publishers</description>
  <maintainer email="teju81@gmail.com">Raviteja U.</maintainer>
  <license>Apache-2.0</license>
  <author email="teju81@gmail.com">Raviteja U.</author>

<!-- executable dependancies needs to be declared over here -->

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```
</details>



<details>

<summary>Barebones setup.py File</summary>

```python
from setuptools import find_packages, setup

package_name = 'multi_robot_py_publisher_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raviteja U.',
    maintainer_email='teju81@gmail.com',
    description='Package for Multi Robot Py Publishers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

</details></br>

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
</br>

## 3 Publishers and Subscribers

Nodes are executable processes that communicate over the ROS graph. In this tutorial, the nodes will pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

### 3.1 Writing a simple publisher and subscriber in C++

#### 3.1.1 Publisher

Below is the code for a minimal publisher.


<details>

<summary>C++ Publisher Code</summary>

```c++
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
</details></br>

#### 3.1.2 Subscriber

Below is the code for a minimal subscriber. Note the ``#include "rclcpp/rclcpp.hpp"`` and ``#include "std_msgs/msg/string.hpp"`` dependencies being added at the beginning of the program. These dependencies will need to be added to the ``package.xml`` file as build dependencies and executable dependencies (look at the ``package.xml`` file here and compare and contrast it with the barebones ``package.xml`` shown earlier in this tutorial).


<details>

<summary>C++ Subscriber Code</summary>

```c++
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
</details></br>

#### 3.1.3 Boiler Plate Code in package files

In order to be able to run the nodes of the package we will first need to add some code to the two package files ``package.xml`` and ``CMakeLists.txt``.

**package.xml**

The highlighted section of the code is what needs to be added to a barebones ``package.xml``. More specifically, the lines with tags ``<build_depend>`` and ``<exec_depend>``.

<details>

<summary>package.xml File</summary>


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
```
```xml
  <build_depend>rclcpp</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>rclcpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```
```
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
</details></br>

**CMakeLists.txt**

Note the highlighted code section. The dependencies associated with the publisher and subscriber implementations are added as ``find_package(<package_name>)``. The executables associated with the nodes of the package are then added. The executable for the publisher ``multi_robot_publisher.cpp`` is called ``talker``. Similarly, The executable for the subscriber ``multi_robot_subscriber.cpp`` is called ``listener``. Note that the executables are mentioned in several places within the file.

<details>



<summary>CMakeLists.txt File</summary>

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

# Find Dependencies
find_package(ament_cmake REQUIRED)
```
```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


# Add Executables

add_executable(talker src/multi_robot_publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/multi_robot_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)
```
```
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
</details></br>



### 3.2 Writing a simple publisher and subscriber in Python

**Publisher**

This is a simple python publisher.

<details>

<summary>Python Publisher Code</summary>

```python
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
</details></br>

**Subscriber**

This is a simple subscriber.

<details>

<summary>Python Subscriber Code</summary>

```python
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
</details></br>

<details>

<summary>package.xml file</summary>

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <!-- Package name must match the project name given in setup.py -->
  <name>multi_robot_py_publisher_package</name>
  <version>1.0.0</version>
  <description>Package for Multi Robot Py Publishers</description>
  <maintainer email="teju81@gmail.com">Raviteja U.</maintainer>
  <license>Apache-2.0</license>
  <author email="teju81@gmail.com">Raviteja U.</author>
```
```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```
```
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

</details></br>

<details>

<summary>setup.py file</summary>

```
from setuptools import find_packages, setup

package_name = 'multi_robot_py_publisher_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raviteja U.',
    maintainer_email='teju81@gmail.com',
    description='Package for Multi Robot Py Publishers',
    license='Apache-2.0',
    tests_require=['pytest'],
```
```python
    entry_points={
        'console_scripts': [
                'talker = multi_robot_py_publisher_package.multi_robot_publisher:main',
                'listener = multi_robot_py_publisher_package.multi_robot_subscriber:main',
        ],
    },
)
```

</details></br>

### 3.3 Remarks on Publishers and Subscribers

- One can run have a python publisher and C++ subscriber (and vice versa).
- There are options one can pass onto while creating a publisher or subscriber. Would be a good idea to experiment with these options at some point. Some example code exists in [2].
- Setting the QoS parameters is the main goal. Look at the code in [3].
  

## 4 Interfaces

Interfaces need to be defined as a package and you need to make sure you are defining it alongside all the other packages already defined. make sure you are in the same workspace as those packages ``multi_robot_ws/src``, and then run the following command to create a new package:

``ros2 pkg create --build-type ament_cmake --license Apache-2.0 multi_robot_interfaces``


Remarks:

- Definitions for custom messages, services and actions
- Note that interfaces can only be a CMake package, but this doesn’t restrict in which type of packages you can use your messages and services. You can create your own custom interfaces in a CMake package, and then use it in a C++ or Python node.


## 6 Services

To be done

## 7 Action Servers

To be done

## 7 References

1. ROS2 Iron getting started
2. Github ROS2 examples in iron branch
3. Differential QoS Thesis

