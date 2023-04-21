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
    CACHE BOOL ON)

set(PLUGIN_ros2-talkerpub
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-tf
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-pcl
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-images
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-laserscan
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-odometry
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-cmdvel
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-navigator
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-dynamic-reconfigure
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-navgraph-breakout
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-joint
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-robot-description
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-move-base
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-skiller
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-position-3d
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-clock
    ON
    CACHE BOOL ON)
set(PLUGIN_ros2-imu
    ON
    CACHE BOOL ON)
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
  depend_on_ros2(ros2-tf)
  depend_on_tf(ros2-tf)
  depend_on_ros2_libs(ros2-tf "tf2;tf2_msgs")
  target_link_libraries(
    ros2-tf
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    fawkestf
    TransformInterface)
  optional_depend_on_ros2_libs(ros2-tf tf2 tfs_deps_found)

  if(tf2_deps_found)
    target_compile_definitions(ros2-tf PUBLIC HAVE_TF2_MSGS)
  else()
    build_skipped_message("ros2-tf tf2 support" "ros2-tf2[-devel]")
  endif()
else()
  plugin_disabled_message(ros2-tf)
endif()

# if(PLUGIN_ros2-pcl) add_library(ros2-pcl MODULE pcl_plugin.cpp pcl_thread.cpp)
# depend_on_ros2(ros2-pcl) depend_on_pcl(ros2-pcl) depend_on_ros2_libs(ros2-pcl
# "sensor_msgs;pcl_conversions;sensor_msgs") target_link_libraries( ros2-pcl
# fawkescore fawkesutils fawkesaspects fawkesblackboard fawkesinterface
# fawkesros2aspect fawkestf fawkespcl_utils) else()
# plugin_disabled_message(ros2-pcl) endif()

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
  depend_on_ros2_libs(ros2-odometry nav_msgs)
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

set(ros2-navigator_DEPS move_base_msgs dynamic_reconfigure actionlib)

if(PLUGIN_ros2-navigator)
  add_library(ros2-navigator MODULE navigator_plugin.cpp navigator_thread.cpp)
  target_link_libraries(ros2-navigator ${Boost_THREAD_LIBRARY})
  depend_on_ros2(ros2-navigator)
  depend_on_boost_libs(ros2-navigator thread)
  depend_on_ros2_libs(ros2-navigator "${ros2-navigator_DEPS}")
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
    NavigatorInterface)
else()
  plugin_disabled_message(ros2-navigator)
endif()

if(PLUGIN_ros2-move-base)
  add_library(ros2-move-base MODULE move_base_plugin.cpp move_base_thread.cpp)
  depend_on_ros2(ros2-move-base)
  depend_on_ros2_libs(ros2-move-base "${ros2-navigator_DEPS}")
  target_link_libraries(ros2-move-base ${Boost_THREAD_LIBRARY})
  target_link_libraries(
    ros2-move-base
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkestf
    fawkesros2aspect
    NavigatorInterface)
else()
  plugin_disabled_message(ros2-move-base)
endif()

if(PLUGIN_ros2-navgraph-breakout)
  add_library(ros2-navgraph-breakout MODULE navgraph_breakout_plugin.cpp
                                            navgraph_breakout_thread.cpp)
  depend_on_ros2(ros2-navgraph-breakout)
  depend_on_ros2_libs(ros2-navgraph-breakout fawkes_msgs)
  target_link_libraries(
    ros2-navgraph-breakout
    m
    pthread
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    NavigatorInterface)
else()
  plugin_disabled_message(ros2-navgraph-breakout)
endif()

if(PLUGIN_ros2-position-3d)
  add_library(ros2-position-3d MODULE position_3d_thread.cpp
                                      position_3d_plugin.cpp)
  depend_on_ros2(ros2-position-3d)
  depend_on_ros2_libs(ros2-position-3d fawkes_msgs)
  target_link_libraries(
    ros2-position-3d
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    Position3DInterface)
else()
  plugin_disabled_message(ros2-position-3d)
endif()

if(PLUGIN_ros2-skiller)
  add_library(ros2-skiller MODULE skiller_plugin.cpp skiller_thread.cpp)
  depend_on_ros2(ros2-skiller)
  depend_on_ros2_libs(ros2-skiller "fawkes_msgs;actionlib")
  target_link_libraries(
    ros2-skiller
    fawkescore
    ros2-navgraph-breakouts
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    SkillerInterface)
else()
  plugin_disabled_message(ros2-skiller)
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
