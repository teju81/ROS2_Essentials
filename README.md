# ROS2 101


This repository will serve as a quick tutorial/recap and also template for your awesome robotics software projects that will be based on ROS2. I am using ROS2 Iron on Ubuntu 22.04 at the time of making this tutorial. I find that the tutorial on ROS2 website is not very clear and also well organized. This is my attempt to try and rectify this.

**Note**

- This tutorial is incomplete, not organized like the way I would like it to be and many parts need to be reworked. This is not yet a replacement for the tutorial on the ROS2 website.
- Defining a custom interface as a package and using it in a seperate package where a topic uses it has been completed (need to extend it to be used by services and action servers).
- Defining a custom interface as a package which builds on another message type needs to be done.



## Skeleton
### ROS Concepts
### 


## 1 ROS2 Concepts

<details>


ROS applications typically communicate through interfaces of one of three types: topics, services, or actions. ROS 2 uses a simplified description language, the interface definition language (IDL), to describe these interfaces. This description makes it easy for ROS tools to automatically generate source code for the interface type in several target languages.


<summary> Details of ROS2 Concepts </summary>


### 1.1 Nodes and Discovery

Nodes exchange information between each other. They are the unit of computation in ROS and each node should be designed to perform one logical thing. There are three types of nodes

1. Topics
2. Services
3. Actions

Nodes advertise their presence to other nodes on the network with the same ROS domain ID. Nodes will only establish connections with other nodes if they have compatible Quality of Service settings.

### 1.2 Interfaces

Each node (whether topics, services, or actions) is associated with (or described by) an interface (either custom or predefined). ROS 2 uses a simplified description language, the interface definition language (IDL), to describe these interfaces. This description makes it easy for ROS tools to automatically generate source code for the interface type in several target languages.

There are three types of interfaces:
- msg: ``.msg`` files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages. They are associated with topics.
- srv: ``.srv`` files describe a service. They are composed of two parts: a request and a response. The request and response are message declarations.
- action: ``.action`` files describe actions. They are composed of three parts: a goal, a result, and feedback. Each part is a message declaration itself.


### 1.2.1 Field Types


| Type Name | C++            | Python          | DDS Type           |
| :-------- | :----------:   | ------------:   |  ---------------:  |
| bool      | bool           | builtins.bool   | boolean            |
| byte      | uint8_t        | builtins.bytes* | octet              |
| char      | char           | builtins.str*   | char               |
| float32   | float          | builtins.float* | float              |
| int8      | int8_t         | builtins.int*   | octet              |
| uint8     | uint8_t        | builtins.int*   | octet              |
| int16     | int16_t        | builtins.int*   | short              |
| uint16    | uint16_t       | builtins.int*   | unsigned short     |
| int32     | int32_t        | builtins.int*   | long               |
| uint32    | uint32_t       | builtins.int*   | unsigned long      |
| int64     | int64_t        | builtins.int*   | long long          |
| uint64    | uint64_t       | builtins.int*   | unsigned long long |
| string    | std::string    | builtins.str    | string             |
| wstring   | std::u16string | builtins.str    | wstring            |


Every built-in-type can be used to define arrays:


| Type Name                | C++                | Python         | DDS Type       |
| :------------------------| :--------------:   | ----:          | ------------:  |
| static array             | std::array<T, N>   | builtins.list* | T[N]           |
| unbounded dynamic array  | std::vector        | builtins.list  | sequence       |
| bounded dynamic array    | custom_class<T, N> | builtins.list* | sequence<T,N>  |
| bounded string           | std::string        | builtins.str*  | string         |

Example of message definition using arrays and bounded types:

```
int32[] unbounded_integer_array
int32[5] five_integers_array
int32[<=5] up_to_five_integers_array

string string_of_unbounded_size
string<=10 up_to_ten_characters_string

string[<=5] up_to_five_unbounded_strings
string<=10[] unbounded_array_of_strings_up_to_ten_characters_each
string<=10[<=5] up_to_five_strings_up_to_ten_characters_each
```


#### 1.2.2 Field Names

- Field names must be lowercase alphanumeric characters with underscores for separating words.
- Field names must start with an alphabetic character, and they must not end with an underscore or have two consecutive underscores.


#### 1.2.3 Field Default Values

Defining a default value is done by adding a third element to the field definition line, i.e:


``fieldtype fieldname fielddefaultvalue``


For example:


```
uint8 x 42
int16 y -2000
string full_name "John Doe"
int32[] samples [-200, -100, 0, 100, 200]
```

#### 1.2.4 Constants

Each constant definition is like a field description with a default value, except that this value can never be changed programatically. This value assignment is indicated by use of an equal ‘=’ sign, e.g.

``constanttype CONSTANTNAME=constantvalue``

Note: Constants names have to be UPPERCASE


For example:

```
int32 X=123
int32 Y=-123
string FOO="foo"
string EXAMPLE='bar'
```


### 1.3 Topics

A publish/subscribe system is one in which there are producers of data (publishers) and consumers of data (subscribers). The publishers and subscribers know how to contact each other through the concept of a “topic”, which is a common name so that the entites can find each other. For instance, when you create a publisher, you must also give it a string that is the name of the topic; the same goes for the subscriber. Any publishers and subscribers that are on the same topic name can directly communicate with each other. There may be zero or more publishers and zero or more subscribers on any particular topic. When data is published to the topic by any of the publishers, all subscribers in the system will receive the data. This system is also known as a “bus”, since it somewhat resembles a device bus from electrical engineering. This concept of a bus is part of what makes ROS 2 a powerful and flexible system.

Topics should be used for continuous data streams (not necessarily at a constant rate??), like sensor data, robot state, etc.

Note: ROS 2 is “anonymous”. This means that when a subscriber gets a piece of data, it doesn’t generally know or care which publisher originally sent it (though it can find out if it wants).

### 1.4 Services

In ROS 2, a service refers to a remote procedure call. In other words, a node can make a remote procedure call to another node which will do a computation and return a result.

In ROS 2, services are expected to return quickly, as the client is generally waiting on the result. Services should never be used for longer running processes, in particular processes that might need to be preempted for exceptional situations. If you have a service that will be doing a long-running computation, consider using an action instead.


A service consists of two parts: the service server and the service client. There can only be one server and multiple clients for a particular service.


### 1.5 Actions

In ROS 2, an action refers to a long-running remote procedure call with feedback and the ability to cancel or preempt the goal. For instance, the high-level state machine running a robot may call an action to tell the navigation subsystem to travel to a waypoint, which may take several seconds (or minutes) to do. Along the way, the navigation subsystem can provide feedback on how far along it is, and the high-level state machine has the option to cancel or preempt the travel to that waypoint.

In ROS 2, actions are expected to be long running procedures, as there is overhead in setting up and monitoring the connection. If you need a short running remote procedure call, consider using a service instead.

An Action consists of two parts: the action server and the action client. There can only be one server and multiple clients for a particular Action.


### 1.6 Parameters

Parameters in ROS 2 are associated with individual nodes. Parameters are used to configure nodes at startup (and during runtime), without changing the code. The lifetime of a parameter is tied to the lifetime of the node (though the node could implement some sort of persistence to reload values after restart).

- Unlike in ROS1 where parameters were served by the roscore, in ROS2 the parameters are associated with a node.
- parameters can be set in 3 ways \
    a) From command line using ``ros param set /<node_name> <parameter_name> <value> \
    b) From launch files \
    c) From YAML files
- Parameters are declared inside the node or the node can be configured to avoid declaring at bringup/spinup of the node.
- Parameter updates have callbacks that can be customized to run with a desired behavior.
- Parameters have descriptions and constraints that can be defined.

**What is the difference between parameter and argument in a launch file node declaration?**
**What is output=screen in a launch file node declaration?**


### 1.7 Launch Files
- Launch Description
- Launch Actions: IncludeLaunchDescription, DeclareLaunchArgument
- Launch ROS Actions: Node PushRosNameSpace
- Launch Substitutions: - Launch configuration, PythonExpression
- Launch Conditions: IfCondition
- Launch Description Sources: PythonLaunchDescriptionSource

**Example of multiple launch files being launched inside a mother launch file**

```
return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_neo.launch.py']),
            condition=IfCondition(PythonExpression(['not ', use_amcl])),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_amcl.launch.py']),
            condition=IfCondition(use_amcl),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_neo.launch.py']),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'params_file': param_dir}.items()),
        ])
```

TBD


**To be completed**

</details>

## 2 Template for Multi Robot ROS2 Software

## 2.1 Create Workspace

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

## 2.2 Packages

### 2.2.1 Creating and Building a Package

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


From the root of the workspace ''multi_robot_ws'' build your package, source it and then run it by executing the following commands
```
colcon build --packages-select <package_name>
source install/setup.bash
ros2 run <package_name> <ros_node_executable>
```

Note: The source install/setup.bash needs to be run everytime you open a new terminal. The package is discoverable only after running this command.


### 2.2.2 Structure of a Package

Packages can be written in C++ or Python. For setup, each type requires some boiler plate code and structure to be followed.

#### 2.2.2.1 C++ Package

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

#### 2.2.2.2 Python Package

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


## 2.3 Interfaces

Interfaces need to be defined as a package and you need to make sure you are defining it alongside all the other packages already defined. make sure you are in the same workspace as those packages ``multi_robot_ws/src``, and then run the following command to create a new package:

``ros2 pkg create --build-type ament_cmake --license Apache-2.0 multi_robot_interfaces``

The .msg, .srv and .action files are required to be placed in directories called msg, srv and action, respectively. Create the directories in ``multi_robot_ws/src/multi_robot_interfaces``:

``mkdir msg srv action``

### 2.3.1 Messages

- create Num.msg and Sphere.msg
- create MultiRobotMessage.msg

### 2.3.2 Remarks on Messages

- One needs to define custom messages in .msg files and place them in the msg directory
- One can define multiple custom messages, each in its own .msg file
- One can define custom messages based off other previously defined custom messages (usually existing example ROS2 interfaces)
- Corresponding hpp files for the .msg files will be generated with all capitals converted into lower case and underscores inserted as a prefix. For example, corresponding to the message MultiRobotMessage.msg the hpp file generated will be multi_robot_message.hpp. This is true for .srv files as well.



## 2.4 Topics, Publishers and Subscribers

Nodes are executable processes that communicate over the ROS graph. In this tutorial, the nodes will pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

### 2.4.1 Writing a simple publisher and subscriber in C++

#### 2.4.1.1 Publisher

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

#### 2.4.1.2 Subscriber

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

#### 2.4.1.3 Boiler Plate Code in package files

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



### 2.4.2 Writing a simple publisher and subscriber in Python

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

### 2.4.3 Remarks on Publishers and Subscribers

- One can run have a python publisher and C++ subscriber (and vice versa).
- There are options one can pass onto while creating a publisher or subscriber. Would be a good idea to experiment with these options at some point. Some example code exists in [2].
- Setting the QoS parameters is the main goal. Look at the code in [3].


### 2.5 Services

- create MultiRobotService.srv
- Requests service by providing 3 int64 inputs a, b, c and service responds with int64 sum


#### 2.5.1 Remarks on Services

- One needs to define custom messages in .msg files and place them in the msg directory
- One can define multiple custom messages, each in its own .msg file
- One can define custom messages based off other previously defined custom messages (usually existing example ROS2 interfaces)


### 2.5.2 CMakeLists.txt




Remarks:

- Definitions for custom messages, services and actions
- Note that interfaces can only be a CMake package, but this doesn’t restrict in which type of packages you can use your messages and services. You can create your own custom interfaces in a CMake package, and then use it in a C++ or Python node.


## 6 Services

To be done

## 7 Action Servers

To be done

## 7 References

1. ROS2 Iron getting started Tutorials
2. Github ROS2 examples in iron branch
3. Differential QoS Thesis
4. ROS2 Iron Concepts

