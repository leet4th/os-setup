## 1 - Configuring ROS 2 environment
# Always need to source environment
source ~/ros2_galactic/ros2-linux/setup.bash

# Check env vars
printenv | grep -i ROS
# Should see the following
#    ROS_VERSION=2
#    ROS_PYTHON_VERSION=3
#    ROS_DISTRO=galactic

## 2 - ROS_DOMAIN_ID
# ROS 2 middleware defaults to DDS. In DDS, the primary mechanism for having different
# logical networks share a physical network is known as Domain ID.
# ROS2 nodes on the same domain can freely discover and send messages to each other, while
# ROS2 nodes on different domains connot.
# Default Domain ID = 0
# Really need to worry about this if different groups of computers are running ROS2 on the same network.
# Then each set should have their own Domain ID.
# Safe numbers choose domain ID between 0 and 101 (inclusive).
#export ROS_DOMAIN_ID=<your_domain_id>
export ROS_DOMAIN_ID=0

## 3 - Turtlesim
# Turtlesim is a lightweight simulator for learning ROS2.
# rqt is a GUI tool for ROS2. Everything done in rqt can be done via command line.

# Configure turtlesim and rqt
sudo apt update
sudo apt install ros-galactic-turtlesim
sudo apt install ~nros-galactic-rqt*

# See installed packages
ros2 pkg executables turtlesim
# Should return
#    turtlesim draw_square
#    turtlesim mimic
#    turtlesim turtle_teleop_key
#    turtlesim turtlesim_node

# Start turtlesim
ros2 run turtlesim turtlesim_node # starts sim node
# Start teleop node to control turtle
ros2 run turtlesim turtle_teleop_key

# ros2 list - see nodes and their associated services, topics, and actions
ros2 node list
ros2 topic list
ros2 service list
ros2 action list

# Bring up rqt. Then go to Plugins > Services > Service Caller
rqt

# Control turtle2 when the turtle2 teleop terminal is open
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel

## 4 - Understanding ROS2 Nodes
# Each node should be responsible for a single, module purpose (controlling wheel motors,
# controlling laster range finder, etc). Each node can send & recieve data to other nodes via topics,
# services, actions, or parameters
# In ROS2 a single executable (C++ program, python program, etc) can contain one or more nodes

# ros2 run command - launches an executable from a package
ros2 run <package_name> <executable_name>
# For example: ros2 run turtlesim turtlesim_node

# ros2 node list
# Outputs the running nodes

# Remapping - allows you to reassign default node properties like node name, topic names, service names, etc to
# custom values. We did this previously by using remapping on turtle_teleop_key to change the default turtle
# being controlled.
# Here we use --remap to reassign the name of our /turtlesim node to /my_turtle
# Since we used ros2 run we open a new window
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

# So now ros2 node list gives:
#   /my_turtle
#   /turtlesim
#   /teleop_turtle

# ros2 node info <node_name> to access more info about the node
# ros2 node info returns a list of subscribers, publishers, services,
# and actions (the ROS graph connections) that interact with that node.
ros2 node info /my_turtle

## 5 - Understanding ROS2 Topics
# Topics act as a bus for nodes to exchange messages
# A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics
# Setup turtlesim again
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

# rqt_graph - gui to visualize the changing nodes and topics and their connections
rqt_graph

# Can get the same info from the command line
ros2 topic list # get list of all topics currently active
ros2 topic list -t # same list but now with topic type appended in brackets

# See data being published on a topic
# ros2 topic echo <topic_name>
ros2 topic echo /turtle1/cmd_vel

# ros2 topic info  - look at what's running with topic
# topics can be point-point, or one-to-many, or many-to-many
ros2 topic info /turtle1/cmd_vel

# Does send data over topics using messages. Publishers and subscribers must send and receive the same type
# of message to communicate.
# ros2 interface show
geometry_msgs/msg/Twist
# means that in the package geometry_msgs there is a msg called Twist
ros2 interface show <msg type> shows the details of the message (structure of message)
ros2 interface show geometry_msgs/msg/Twist

# Now that we know the message structure, we can publish data onto a topic directly using command line
ros2 topic pub <topic_name> <msg_type> '<args>'
# where '<args>' is the actual data to pass. This must be in YAML syntax
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# Note --once sense this command once then stops
# This creates a stream of commands
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
# --rate 1 option tells ros2 topic pub to publish a steady stream at 1Hz

# Look at the turtle's pose
ros2 topic echo /turtle1/pose

# View rate at which data is published
ros2 topic hz /turtle1/pose
ros2 topic hz /turtle1/cmd_vel # note this will be at 1hz becuase we're publishing at 1hz manually!

## 6 - Understanding ROS2 Services
# Another method of communicatoin for nodes in the ROS graph.
# Services are based on call-and-response model
# Services only provide data when they are specifically called by a client
# Client - request -> Server - response

# Setup turtlesim again
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
# Get list of all active serivces
ros2 service list
# /clear
# /kill
# /reset
# /spawn
# /teleop_turtle/describe_parameters
# /teleop_turtle/get_parameter_types
# /teleop_turtle/get_parameters
# /teleop_turtle/list_parameters
# /teleop_turtle/set_parameters
# /teleop_turtle/set_parameters_atomically
# /turtle1/set_pen
# /turtle1/teleport_absolute
# /turtle1/teleport_relative
# /turtlesim/describe_parameters
# /turtlesim/get_parameter_types
# /turtlesim/get_parameters
# /turtlesim/list_parameters
# /turtlesim/set_parameters
# /turtlesim/set_parameters_atomically

# Note both nodes have the same 6 services with parameters in their names
# Nearly every node in ROS2 has these infrastructre services that parameters are built off of
/<node>/describe_parameters
/<node>/get_parameters
/<node>/list_parameters
/<node>/set_parameters
/<node>/set_parameters_atomically

# Find out service type
ros2 service type <service_name>
ros2 service type /clear
# std_srvs/srv/Empty
# Empty means the service call sends no data when making a request and receives no data when
# receiving a response
ros2 service list -t # -t is short of --show-types

# Find all services of a certain type
ros2 service find <type_name>
ros2 service find std_srvs/srv/Empty
# Returns
#    /clear
#    /reset
# Again use interface show to give structure of message
ros2 interface show <type_name>
# If no message like for /clear (because type is std_srvs/srv/Empty) returns ---
# Call a service from command line
ros2 service call <service_name> <service_type> <arguments>
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"

## 7 - Understanding ROS2 Parameters
# A parameter is a configuration value of a node. Can think of it as node settings.
# Used to config nodes at startup and during runtime without changing code.
# Each parameter consists of a key, a value, and a descriptor.
# The key is a string and the value is one of the following types:
#     bool, int64, float64, string, byte[], bool[], int64[], float64[] or string[]
# In ROS2, each node maintains its own parameters
# List params
ros2 param list
# Every node has the parameter "use_sim_time"
# Get parameter value
ros2 param get <node_name> <parameter_name>
ros2 param get /turtlesim background_g
# Set parameter value
ros2 param set <node_name> <parameter_name> <value>
ros2 param set /turtlesim background_r 150
# Dump all node's current parameter values to a file to save for later
# Handy for when you want to reload the node with the same parameters in the future
ros2 param dump <node_name>
ros2 param dump /turtlesim
#    Saving to:  ./turtlesim.yaml
# Loading parameters
ros2 param load <node_name> <parameter_file>
ros2 param load /turtlesim ./turtlesim.yaml
# Load param file on node startup
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml

## 8 - Understanding ROS2 Actions
# Another communication type in ROS2 intended for long running tasks
# Consist of three parts: goal, feedback, result
# Built on topics and services. Funcionality is similar to services, except actions can be canceled.
# They also provide steady feedback as opposed to services which return a single response
# Actions use client-server model, similar to pub-sub model.
# Action client node sends goal to action server node
# Action server node acknowledges goal and returns stream of feedback and a result
# List actoins
ros2 action list
# Info
ros2 action info <action_name>
ros2 action info /turtle1/rotate_absolute
# Interface
ros2 interface show <action_type>
ros2 interface show turtlesim/action/RotateAbsolute
# Send a goal (add --feedback to see feedback)
ros2 action send_goal <action_name> <action_type> <values>
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}" --feedback

## 9 - Using rqt_console
# rqt_console is GUI to introspect log messages in ROS2. Typcially, log msgs show up in terminal.
# With rqt_console, you can collect messages over time, view them, filter them, save them, etc.
#There is no exact standard for what each level indicates, but it’s safe to assume that:
# Fatal messages indicate the system is going to terminate to try to protect itself from detriment.
# Error messages indicate significant issues that won’t necessarily damage the system,
#    but are preventing it from functioning properly.
# Warn messages indicate unexpected activity or non-ideal results that might represent a
#    deeper issue, but don’t harm functionality outright.
# Info messages indicate event and status updates that serve as a visual verification that the system
#    is running as expected.
# Debug messages detail the entire step-by-step process of the system execution.
# You can set the default logger level when you first run the /turtlesim node using remapping.
# Enter the following command in your terminal:
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
# Now you won’t see the initial Info level warnings that came up in the console last time you
# started turtlesim. That’s because Info messages are lower priority than the new default severity, Warn.

## 10 - Intro to ROS2 Launch
# Command line tool to launch multiple nodes at once
ros2 launch turtlesim multisim.launch.py
# This runs the following launch file written in python
# Launch files can written using python, xml, and YAML
#    # turtlesim/launch/multisim.launch.py
#
#    from launch import LaunchDescription
#    import launch_ros.actions
#
#    def generate_launch_description():
#        return LaunchDescription([
#            launch_ros.actions.Node(
#                namespace= "turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
#            launch_ros.actions.Node(
#                namespace= "turtlesim2", package='turtlesim', executable='turtlesim_node', output='screen'),
#        ])

## 11 - Recording and playback
# Use ros2 bag to record data published to a topic
ros2 bag record <topic_name>
ros2 bag record /turtle1/cmd_vel
# Record multiple topics
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose

# See details about recording
ros2 bag info <bag_file_name>
ros2 bag info subset

# Replay
ros2 bag play <bag_file_name>
ros2 bag play subset

# 12 - Creating a workspace
# Workspace is a dir containig ROS2 packages.
# In ROS2, you need to source your ROS2 installation workspace in the terminal you plan to work in. This
# makes ROS2 packages available for you to use
# You can also source an "overlay" or secondary workspace where you can add new packages without interfering
# with the existing ROS2 workspace that you're extending ("underlay")
# Packages in overlay will override packages in the underlay
# Possible to have several layers of underlays and overlays with each successive overlay using the packages
# of its parent underlay
# Workflow
# 1 - source the underlay (ros2 installation in this case)
# 2 - Create a new dir
# 3 - Develop package or clone a repo
# 4 - Resolve dependencies
# 5 - Build workspace with colcon
# 6 - Source the overly (in new terminal)

# 1 - source the underlay (ros2 installation in this case)
source ~/ros2_galactic/ros2-linux/setup.bash

# 2 - Create a new dir
# Best practice is to create a new dir for every new workspace. Name doesnt matter, but it's helpful to indicate
# the purpose of the workspace.
# Another best practice is to put any packages in your workspace into src dir.
mkdir -p dev_ws/src
cd dev_ws/src

# 3 - Clone a smaple repo
# Clone into dev_ws/src. Note we are specifying the branch for ros2 galactic
# This repo contains the turtlesim package. The other packages in this repo are not built because they contain
# a COLCON_IGNORE file
git clone https://github.com/ros/ros_tutorials.git -b galactic-devel

# 4 - Resolve dependencies
# Before building the workspace, need to resolve package dependencies. May already have them but best practice
# is to check for dependencies every time you clone. Otherwise build may fail after you waited
# Packages declare their dependencies in the package.xml file.
# Run from root of the workspace (dev_ws)
rosdep install -i --from-path src --rosdistro galactic -y

# 5 - Build workspace with colcon
# Build from root of your workspace (dev_ws)
colcon build
# Some useful args to colcon build
#     --packages-up-to builds the package you want, plus all its dependencies,
#             but not the whole workspace (saves time)
#     --symlink-install saves you from having to rebuild every time you tweak python scripts
#     --event-handlers console_direct+ shows console output while building (can otherwise be found in the log directory)

# 6 - Source the overly (in new terminal)
# Best to create a new terminal and not source from the same terminal used to build the workspace
# Sourcing the local_setup of the overlay will only add the packages available in the overlay to your
# environment. setup sources the overlay as well as the underlay it was created in, allowing you to
# utilize both workspaces.
# So, sourcing your main ROS 2 installation’s setup and then the dev_ws overlay’s local_setup,
# like you just did, is the same as just sourcing dev_ws’s setup, because that includes the environment
# of the underlay it was created in.
source ~/ros2_galactic/ros2-linux/setup.bash
source dev_ws/install/local_setup.bash
# same as above becuse this contains the environment the underlay was created in
source dev_ws/install/setup.bash

## 12 - Creating your first ROS2 package
# Create a new package using either CMake or Python
# A package can be considered a container for your ROS2 code. To install or share with other, need it to be
# organized in a package.
# Package creation in ROS2 uses ament as build system and colcon as build tool.
# You can create a package using either CMake or Python

# Min required contents:

# CMake package:
#    package.xml file containing meta information about the package
#    CMakeLists.txt file that describes how to build the code within the package
#
#    File structure
#    my_package/
#         CMakeLists.txt
#         package.xml

# Python package:
#    package.xml file containing meta information about the package
#    setup.py containing instructions for how to install the package
#    setup.cfg is required when a package has executables, so ros2 run can find them
#    /<package_name> - a directory with the same name as your package, used by ROS 2 tools to find your
#                      package, contains __init__.py
#
#    File structure
#    my_package/
#          setup.py
#          package.xml
#          resource/my_package

# Packages in a workspace
# A single workspace can contain as many packages as you want, each in their own folder
# Can also have different build types in one workspace (CMake, Python)
# Cannot have nested packages
# Best practice: have a src folder within workspace that contains all packages
#
# Example workspace:
#     workspace_folder/
#         src/
#           package_1/
#               CMakeLists.txt
#               package.xml
#
#           package_2/
#               setup.py
#               package.xml
#               resource/package_2
#           ...
#           package_n/
#               CMakeLists.txt
#               package.xml

# Creating a package
source ~/ros2_galactic/ros2-linux/setup.bash
cd dev_ws/src
# 1 - Create
# Note only difference is in specified build type
# CMake
ros2 pkg create --build-type ament_cmake <package_name>
# Python
ros2 pkg create --build-type ament_python <package_name>
# Can alos use optional arg --node-name <node_name> to create a simple Hello World type executable in package
ros2 pkg create --build-type ament_cmake --node-name my_node my_package_cpp
ros2 pkg create --build-type ament_python --node-name my_node my_package_py

# 2 - Build
cd dev_ws
colcon build
# or to just build specified packages
colcon build --packages-select my_package_cpp my_package_py


## 13 - Writing simple pub/sub package in C++
# Create the package
ros2 pkg create --build-type ament_cmake cpp_pubsub

# Download the file to the package's src dir
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_publisher/member_function.cpp

# Add dependencies and the description, maintainer, license fields
# The package needs rclcpp and std_msgs
# package.xml needs
#    <description>Examples of minimal publisher/subscriber using rclcpp</description>
#    <maintainer email="you@email.com">Your Name</maintainer>
#    <license>Apache License 2.0</license>
#
# and after <buildtool_depend>ament_cmake</buildtool_depend>
#
#    <depend>rclcpp</depend>
#    <depend>std_msgs</depend>

# CMakeLists.txt needs
#    find_package(rclcpp REQUIRED)
#    find_package(std_msgs REQUIRED)
#
#    # Add the executable and name it talker so you can run node using ros2 run
#    add_executable(talker src/publisher_member_function.cpp)
#    ament_target_dependencies(talker rclcpp std_msgs)
#
#    # Add install(TARGETS...) section so ros2 run can find the executable
#    install(TARGETS
#      talker
#      DESTINATION lib/${PROJECT_NAME})

# Return to dev_ws/src/cpp_pubsub/src to create the next node in this package
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_subscriber/member_function.cpp
# Looking at code, recall that the topic name and message type used by the pub & sub must match
# MinimalSubscriber has the same dependencies as the publisher node so nothing new to add to package.xml

# CMakeLists.txt needs
#    add_executable(listener src/subscriber_member_function.cpp)
#    ament_target_dependencies(listener rclcpp std_msgs)
#
#    # Add listener to install list
#    install(TARGETS
#      talker
#      listener
#      DESTINATION lib/${PROJECT_NAME})

# Build and run
# We likely already have rclcpp and std_msgs packages installed but good practice to run rosdep in the root
# of our workspace (dev_ws) to check for missing dependencies before building
rosdep install -i --from-path src --rosdistro galactic -y

# Quickly just build our new package
colcon build --packages-select cpp_pubsub

# Open new terminal and run
source install/setup.bash
ros2 run cpp_pubsub talker
ros2 run cpp_pubsub listener

## 14 - Writing simple pub/sub package in Python
# Create package
ros2 pkg create --build-type ament_python py_pubsub

# Download example talker code into package python package dir
# In dev_ws/src/py_pubsub/py_pubsub
# Get pub file
wget https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
# Get sub file
wget https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py

# Add to package.xml
#    <description>Examples of minimal publisher/subscriber using rclpy</description>
#    <maintainer email="you@email.com">Your Name</maintainer>
#    <license>Apache License 2.0</license>
#
#    <exec_depend>rclpy</exec_depend>
#    <exec_depend>std_msgs</exec_depend>

# Add to setup.py
#    maintainer='YourName',
#    maintainer_email='you@email.com',
#    description='Examples of minimal publisher/subscriber using rclpy',
#    license='Apache License 2.0',
#
#    entry_points={
#            'console_scripts': [
#                    'talker = py_pubsub.publisher_member_function:main',
#                    'listener = py_pubsub.subscriber_member_function:main',
#            ],
#    },

# Check setup.cfg
# This is telling setuptools to put your executables in lib because ros2 run will look for them there
# Should have the following already populated
#    [develop]
#    script-dir=$base/lib/py_pubsub
#    [install]
#    install-scripts=$base/lib/py_pubsub

# Always check for dependencies before building. Good practice
rosdep install -i --from-path src --rosdistro galactic -y

# Build just this package
colcon build --packages-select py_pubsub

## 15 - Writing simple service and client in C++
# Create the service package in src
# Run from dev_ws/src
ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces
# Note that now we use --dependencies to auto populate the package.xml CMakeLists.txt files
# example_interfaces is package that includes the .srv file needed to structure requests/responses
#    int64 a
#    int64 b
#    ---
#    int64 sum

# Update package.xml
# Only need to do description & license because --dependencies did the rest for us

# Write code for add_two_ints_server.cpp and add_two_ints_client.cpp in package src

# Add to CMakeLists.txt to create executable
#    add_executable(server src/add_two_ints_server.cpp)
#    ament_target_dependencies(server rclcpp example_interfaces)
#
#    install(TARGETS
#      server
#      client
#      DESTINATION lib/${PROJECT_NAME})

# Check for dependencies
rosdep install -i --from-path src --rosdistro galactic -y


## 16 - Writing simple service and client in Python
# Create package
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces

# Update package.xml description & license
# Update setup.py description & license
# Can also set the entry points we're going to use in setup.py
#    entry_points={
#        'console_scripts': [
#            'service = py_srvcli.service_member_function:main',
#            'client = py_srvcli.client_member_function:main',
#        ],
#    },

# In dev_ws/src/py_srvcli/py_srvcli write service_member_function.py and client_member_function.py

# Check for dependencies
rosdep install -i --from-path src --rosdistro galactic -y

# Build
colcon build --packages-select py_srvcli
