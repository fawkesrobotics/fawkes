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

set(PLUGIN_ros
    ON
    CACHE BOOL "Build ros plugin")
set(PLUGIN_ros-talkerpub
    ON
    CACHE BOOL "Build ros-talkerpub plugin")
set(PLUGIN_ros-tf
    ON
    CACHE BOOL "Build ros-tf plugin")
set(PLUGIN_ros-pcl
    ON
    CACHE BOOL "Build ros-pcl plugin")
set(PLUGIN_ros-images
    ON
    CACHE BOOL "Build ros-images plugin")
set(PLUGIN_ros-laserscan
    ON
    CACHE BOOL "Build ros-laserscan plugin")
set(PLUGIN_ros-odometry
    ON
    CACHE BOOL "Build ros-odometry plugin")
set(PLUGIN_ros-cmdvel
    ON
    CACHE BOOL "Build ros-cmdvel plugin")
set(PLUGIN_ros-navigator
    ON
    CACHE BOOL "Build ros-navigator plugin")
set(PLUGIN_ros-dynamic-reconfigure
    ON
    CACHE BOOL "Build ros-dynamic-reconfigure plugin")
set(PLUGIN_ros-navgraph-breakout
    ON
    CACHE BOOL "Build ros-navgraph-breakout plugin")
set(PLUGIN_ros-joint
    ON
    CACHE BOOL "Build ros-joint plugin")
set(PLUGIN_ros-robot-description
    ON
    CACHE BOOL "Build ros-robot-description plugin")
set(PLUGIN_ros-move-base
    ON
    CACHE BOOL "Build ros-move-base plugin")
set(PLUGIN_ros-skiller
    ON
    CACHE BOOL "Build ros-skiller plugin")
set(PLUGIN_ros-position-3d
    ON
    CACHE BOOL "Build ros-position-3d plugin")
set(PLUGIN_ros-clock
    ON
    CACHE BOOL "Build ros-clock plugin")
set(PLUGIN_ros-imu
    ON
    CACHE BOOL "Build ros-imu plugin")

add_subdirectory(aspect)
if(PLUGIN_ros)

  add_library(ros MODULE ros_plugin.cpp node_thread.cpp)
  depend_on_ros(ros)
  target_link_libraries(
    ros
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect)
else()
  plugin_disabled_message(ros)
endif()

if(PLUGIN_ros-talkerpub)
  add_library(ros-talkerpub MODULE talkerpub_plugin.cpp talkerpub_thread.cpp)
  depend_on_ros(ros-talkerpub)
  depend_on_ros_libs(ros-talkerpub std_msgs)
  target_link_libraries(
    ros-talkerpub
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect)
else()
  plugin_disabled_message(ros-talkerpub)
endif()

if(PLUGIN_ros-tf)
  add_library(ros-tf MODULE tf_plugin.cpp tf_thread.cpp)
  depend_on_ros(ros-tf)
  depend_on_tf(ros-tf)
  depend_on_ros_libs(ros-tf tf)
  target_link_libraries(
    ros-tf
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect
    fawkestf
    TransformInterface)
  optional_depend_on_ros_libs(ros-tf tf2 tf2_deps_found)
  if(tf2_deps_found)
    target_compile_definitions(ros-tf PUBLIC HAVE_TF2_MSGS)
  else()
    build_skipped_message("ros-tf tf2 support" "ros-tf2[-devel]")
  endif()
else()
  plugin_disabled_message(ros-tf)
endif()

if(PLUGIN_ros-pcl)
  add_library(ros-pcl MODULE pcl_plugin.cpp pcl_thread.cpp)
  depend_on_ros(ros-pcl)
  depend_on_pcl(ros-pcl)
  depend_on_ros_libs(ros-pcl "sensor_msgs;pcl_conversions;sensor_msgs")
  target_link_libraries(
    ros-pcl
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect
    fawkestf
    fawkespcl_utils)
else()
  plugin_disabled_message(ros-pcl)
endif()

if(PLUGIN_ros-images)
  add_library(ros-images MODULE image_plugin.cpp image_thread.cpp)
  depend_on_ros(ros-images)
  depend_on_ros_libs(ros-images sensor_msgs)
  target_link_libraries(
    ros-images
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect
    fvutils)
else()
  plugin_disabled_message(ros-images)
endif()

if(PLUGIN_ros-laserscan)
  add_library(ros-laserscan MODULE laserscan_plugin.cpp laserscan_thread.cpp)
  depend_on_ros(ros-laserscan)
  depend_on_ros_libs(ros-laserscan sensor_msgs)
  target_link_libraries(
    ros-laserscan
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect
    fvutils
    Laser360Interface
    Laser720Interface
    Laser1080Interface)
else()
  plugin_disabled_message(ros-laserscan)
endif()

if(PLUGIN_ros-joint)
  add_library(ros-joint MODULE joint_plugin.cpp joint_thread.cpp)
  depend_on_ros(ros-joint)
  depend_on_ros_libs(ros-joint sensor_msgs)
  target_link_libraries(
    ros-joint
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect
    JointInterface)
else()
  plugin_disabled_message(ros-joint)
endif()

if(PLUGIN_ros-imu)
  add_library(ros-imu imu_plugin.cpp imu_thread.cpp)
  depend_on_ros(ros-imu)
  depend_on_ros_libs(ros-imu sensor_msgs)
  target_link_libraries(
    ros-imu
    MODULE
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect
    IMUInterface)
else()
  plugin_disabled_message(ros-imu)
endif()

if(PLUGIN_ros-odometry)
  add_library(ros-odometry MODULE odometry_plugin.cpp odometry_thread.cpp)
  depend_on_ros(ros-odometry)
  depend_on_ros_libs(ros-odometry nav_msgs)
  target_link_libraries(
    ros-odometry
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkestf
    fawkesrosaspect
    MotorInterface)
else()
  plugin_disabled_message(ros-odometry)
endif()

if(PLUGIN_ros-cmdvel)
  add_library(ros-cmdvel MODULE cmdvel_thread.cpp cmdvel_plugin.cpp)
  depend_on_ros(ros-cmdvel)
  depend_on_ros_libs(ros-cmdvel geometry_msgs)
  target_link_libraries(
    ros-cmdvel
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect
    MotorInterface)
else()
  plugin_disabled_message(ros-cmdvel)
endif()

if(PLUGIN_ros-dynamic-reconfigure)
  add_library(ros-dynamic-reconfigure MODULE dynamic_reconfigure_plugin.cpp
                                             dynamic_reconfigure_thread.cpp)
  depend_on_ros(ros-dynamic-reconfigure)
  depend_on_ros_libs(ros-dynamic-reconfigure dynamic_reconfigure)
  target_link_libraries(
    ros-dynamic-reconfigure
    m
    pthread
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect
    DynamicReconfigureInterface)
else()
  plugin_disabled_message(ros-dynamic-reconfigure)
endif()

set(ROS_NAVIGATOR_DEPS move_base_msgs dynamic_reconfigure actionlib)
if(PLUGIN_ros-navigator)
  add_library(ros-navigator MODULE navigator_plugin.cpp navigator_thread.cpp)
  target_link_libraries(ros-navigator ${Boost_THREAD_LIBRARY})
  depend_on_ros(ros-navigator)
  depend_on_boost_libs(ros-navigator thread)
  depend_on_ros_libs(ros-navigator "${ROS_NAVIGATOR_DEPS}")
  target_link_libraries(
    ros-navigator
    m
    pthread
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect
    NavigatorInterface)
else()
  plugin_disabled_message(ros-navigator)
endif()

if(PLUGIN_ros-move-base)
  add_library(ros-move-base MODULE move_base_plugin.cpp move_base_thread.cpp)
  depend_on_ros(ros-move-base)
  depend_on_ros_libs(ros-move-base "${ROS_NAVIGATOR_DEPS}")
  target_link_libraries(ros-move-base ${Boost_THREAD_LIBRARY})
  target_link_libraries(
    ros-move-base
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkestf
    fawkesrosaspect
    NavigatorInterface)
else()
  plugin_disabled_message(ros-move-base)
endif()

if(PLUGIN_ros-navgraph-breakout)
  add_library(ros-navgraph-breakout MODULE navgraph_breakout_plugin.cpp
                                           navgraph_breakout_thread.cpp)
  depend_on_ros(ros-navgraph-breakout)
  depend_on_ros_libs(ros-navgraph-breakout fawkes_msgs)
  target_link_libraries(
    ros-navgraph-breakout
    m
    pthread
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect
    NavigatorInterface)
else()
  plugin_disabled_message(ros-navgraph-breakout)
endif()

if(PLUGIN_ros-position-3d)
  add_library(ros-position-3d MODULE position_3d_thread.cpp
                                     position_3d_plugin.cpp)
  depend_on_ros(ros-position-3d)
  depend_on_ros_libs(ros-position-3d fawkes_msgs)
  target_link_libraries(
    ros-position-3d
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect
    Position3DInterface)
else()
  plugin_disabled_message(ros-position-3d)
endif()

if(PLUGIN_ros-skiller)
  add_library(ros-skiller MODULE skiller_plugin.cpp skiller_thread.cpp)
  depend_on_ros(ros-skiller)
  depend_on_ros_libs(ros-skiller "fawkes_msgs;actionlib")
  target_link_libraries(
    ros-skiller
    fawkescore
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect
    SkillerInterface)
else()
  plugin_disabled_message(ros-skiller)
endif()

if(PLUGIN_ros-clock)
  add_library(ros-clock clock_plugin.cpp clock_thread.cpp)
  depend_on_ros(ros-clock)
  depend_on_ros_libs(ros-clock rosgraph_msgs)
  target_link_libraries(
    ros-clock
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect)

else()
  plugin_disabled_message(ros-clock)
endif()

if(PLUGIN_ros-robot-description)
  add_library(ros-robot-description MODULE robot_description_plugin.cpp
                                           robot_description_thread.cpp)
  depend_on_ros(ros-robot-description)
  target_link_libraries(ros-robot-description fawkescore fawkesutils
                        fawkesaspects fawkesrosaspect)

else()
  plugin_disabled_message(ros-robot-description)
endif()
