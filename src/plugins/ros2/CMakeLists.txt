# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann and Daniel Swoboda
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************

set(PLUGIN_ros2
    ON
    CACHE BOOL "Build ros2 plugin")
set(PLUGIN_ros2-talkerpub
    ON
    CACHE BOOL "Build ros2-talkerpub plugin")
set(PLUGIN_ros2-tf2
    ON
    CACHE BOOL "Build ros2-tf2 plugin")
set(PLUGIN_ros2-images
    ON
    CACHE BOOL "Build ros2-images plugin")
set(PLUGIN_ros2-laserscan
    ON
    CACHE BOOL "Build ros2-laserscan plugin")
set(PLUGIN_ros2-odometry
    ON
    CACHE BOOL "Build ros2-odometry plugin")
set(PLUGIN_ros2-cmdvel
    ON
    CACHE BOOL "Build ros2-cmdvel plugin")
set(PLUGIN_ros2-navigator
    ON
    CACHE BOOL "Build ros2-navigator plugin")
set(PLUGIN_ros2-dynamic-reconfigure
    ON
    CACHE BOOL "Build ros2-dynamic-reconfigure plugin")
set(PLUGIN_ros2-joint
    ON
    CACHE BOOL "Build ros2-joint plugin")
set(PLUGIN_ros2-robot-description
    ON
    CACHE BOOL "Build ros2-robot-description plugin")
set(PLUGIN_ros2-move-base
    ON
    CACHE BOOL "Build ros2-move-base plugin")
set(PLUGIN_ros2-clock
    ON
    CACHE BOOL "Build ros2-clock plugin")
set(PLUGIN_ros2-imu
    ON
    CACHE BOOL "Build ros2-imu plugin")
add_subdirectory(aspect)

if(PLUGIN_ros2)
  add_library(ros2 MODULE ros2_plugin.cpp node_thread.cpp)
  depend_on_ros2(ros2)
  depend_on_ros2_libs(ros2 "rmw;rosidl_typesupport_interface;rcl_interfaces")
  target_link_libraries(
    ros2
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect)
else()
  plugin_disabled_message(ros2)
endif()

if(PLUGIN_ros2-talkerpub)
  add_library(ros2-talkerpub MODULE talkerpub_plugin.cpp talkerpub_thread.cpp)
  depend_on_ros2(ros2-talkerpub)
  depend_on_ros2_libs(ros2-talkerpub "std_msgs")
  target_link_libraries(
    ros2-talkerpub
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect)
else()
  plugin_disabled_message(ros2-talkerpub)
endif()

if(PLUGIN_ros2-tf2)
  add_library(ros2-tf2 MODULE tf2_plugin.cpp tf2_thread.cpp)
  depend_on_ros2(ros2-tf2)
  depend_on_tf(ros2-tf2)
  depend_on_ros2_libs(ros2-tf2 "tf2;tf2_msgs")
  target_link_libraries(
    ros2-tf2
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    fawkestf
    TransformInterface)
  depend_on_ros2_libs(ros2-tf2 tf2 tfs_deps_found)

  if(tf2_deps_found)
    target_compile_definitions(ros2-tf PUBLIC HAVE_TF2_MSGS)
  else()
    build_skipped_message("ros2-tf tf2 support" "ros2-tf2[-devel]")
  endif()
else()
  plugin_disabled_message(ros2-tf2)
endif()

if(PLUGIN_ros2-images)
  add_library(ros2-images MODULE image_plugin.cpp image_thread.cpp)
  depend_on_ros2(ros2-images)
  depend_on_ros2_libs(ros2-images "sensor_msgs;image_transport")
  target_link_libraries(
    ros2-images
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    fvutils)
else()
  plugin_disabled_message(ros2-images)
endif()

if(PLUGIN_ros2-laserscan)
  add_library(ros2-laserscan MODULE laserscan_plugin.cpp laserscan_thread.cpp)
  depend_on_ros2(ros2-laserscan)
  depend_on_ros2_libs(ros2-laserscan sensor_msgs)
  target_link_libraries(
    ros2-laserscan
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    fvutils
    Laser360Interface
    Laser720Interface
    Laser1080Interface)
else()
  plugin_disabled_message(ros2-laserscan)
endif()

if(PLUGIN_ros2-joint)
  add_library(ros2-joint MODULE joint_plugin.cpp joint_thread.cpp)
  depend_on_ros2(ros2-joint)
  depend_on_ros2_libs(ros2-joint sensor_msgs)
  target_link_libraries(
    ros2-joint
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    JointInterface)
else()
  plugin_disabled_message(ros2-joint)
endif()

if(PLUGIN_ros2-imu)
  add_library(ros2-imu imu_plugin.cpp imu_thread.cpp)
  depend_on_ros2(ros2-imu)
  depend_on_ros2_libs(ros2-imu sensor_msgs)
  target_link_libraries(
    ros2-imu
    MODULE
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    IMUInterface)
else()
  plugin_disabled_message(ros2-imu)
endif()

if(PLUGIN_ros2-odometry)
  add_library(ros2-odometry MODULE odometry_plugin.cpp odometry_thread.cpp)
  depend_on_ros2(ros2-odometry)
  depend_on_ros2_libs(ros2-odometry "nav_msgs;rosidl_typesupport_cpp")
  target_link_libraries(
    ros2-odometry
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkestf
    fawkesros2aspect
    MotorInterface)
else()
  plugin_disabled_message(ros2-odometry)
endif()

if(PLUGIN_ros2-cmdvel)
  add_library(ros2-cmdvel MODULE cmdvel_thread.cpp cmdvel_plugin.cpp)
  depend_on_ros2(ros2-cmdvel)
  depend_on_ros2_libs(ros2-cmdvel geometry_msgs)
  target_link_libraries(
    ros2-cmdvel
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    MotorInterface)
else()
  plugin_disabled_message(ros2-cmdvel)
endif()

if(PLUGIN_ros2-dynamic-reconfigure)
  add_library(ros2-dynamic-reconfigure MODULE dynamic_reconfigure_plugin.cpp
                                              dynamic_reconfigure_thread.cpp)
  depend_on_ros2(ros2-dynamic-reconfigure)
  depend_on_ros2_libs(ros2-dynamic-reconfigure dynamic_reconfigure)
  target_link_libraries(
    ros2-dynamic-reconfigure
    m
    pthread
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    DynamicReconfigureInterface)
else()
  plugin_disabled_message(ros2-dynamic-reconfigure)
endif()

set(ROS_2_NAVIGATOR_DEPS nav2_msgs rclcpp_action)

if(PLUGIN_ros2-navigator)
  add_library(ros2-navigator MODULE navigator_plugin.cpp navigator_thread.cpp)
  target_link_libraries(ros2-navigator ${Boost_THREAD_LIBRARY})
  depend_on_ros2(ros2-navigator)
  depend_on_tf(ros2-navigator)
  depend_on_boost_libs(ros2-navigator thread)
  depend_on_ros2_libs(ros2-navigator "${ROS_2_NAVIGATOR_DEPS}")
  target_link_libraries(
    ros2-navigator
    m
    pthread
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    Position3DInterface
    NavigatorInterface)
else()
  plugin_disabled_message(ros2-navigator)
endif()

if(PLUGIN_ros2-clock)
  add_library(ros2-clock clock_plugin.cpp clock_thread.cpp)
  depend_on_ros2(ros2-clock)
  depend_on_ros2_libs(ros2-clock ros2graph_msgs)
  target_link_libraries(
    ros2-clock
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect)

else()
  plugin_disabled_message(ros2-clock)
endif()

if(PLUGIN_ros2-robot-description)
  add_library(ros2-robot-description MODULE robot_description_plugin.cpp
                                            robot_description_thread.cpp)
  depend_on_ros2(ros2-robot-description)
  target_link_libraries(ros2-robot-description fawkescore fawkesutils
                        fawkesaspects fawkesros2aspect)

else()
  plugin_disabled_message(ros2-robot-description)
endif()
