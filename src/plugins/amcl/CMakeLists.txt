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

set(PLUGIN_amcl
    ON
    CACHE BOOL "Build amcl plugin")
set(PLUGIN_amcl2
    ON
    CACHE BOOL "Build amcl2 plugin")
set(PLUGIN_map-lasergen
    ON
    CACHE BOOL "Build map-lasergen plugin")

if(PLUGIN_amcl)
  add_library(amcl MODULE amcl_plugin.cpp amcl_thread.cpp)
  depend_on_tf(amcl)
  set(ROS_LIBS geometry_msgs nav_msgs)
  if(ROS_1_FOUND)
    target_sources(amcl PRIVATE ros_thread.cpp)
    depend_on_ros(amcl)
    depend_on_ros_libs(amcl "${ROS_LIBS}")
  endif()
  target_compile_definitions(amcl PUBLIC USE_ASSERT_EXCEPTION USE_MAP_PUB)
  target_link_libraries(
    amcl
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    fawkesbaseapp
    Laser360Interface
    Position3DInterface
    LocalizationInterface
    fawkes_amcl_pf
    fawkes_amcl_map
    fawkes_amcl_sensors
    fawkes_amcl_utils
    fawkestf)
else()
  plugin_disabled_message(amcl)
endif()

if(PLUGIN_map-lasergen)
  add_library(map-lasergen MODULE map_lasergen_plugin.cpp
                                  map_lasergen_thread.cpp)
  depend_on_tf(map-lasergen)
  target_compile_definitions(map-lasergen PUBLIC USE_ASSERT_EXCEPTION
                                                 USE_MAP_PUB)
  target_link_libraries(
    map-lasergen
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    fawkesbaseapp
    Laser360Interface
    Position3DInterface
    fawkes_amcl_map
    fawkes_amcl_utils
    fawkestf)
else()
  plugin_disabled_message(map-lasergen)
endif()

if(PLUGIN_amcl2)
  add_library(amcl2 MODULE amcl2_plugin.cpp amcl2_thread.cpp)
  depend_on_tf(amcl2)
  set(ROS_LIBS geometry_msgs nav_msgs)
  if(ROS_2_FOUND)
    target_sources(amcl2 PRIVATE ros2_thread.cpp)
    depend_on_ros2(amcl2)
    depend_on_ros2_libs(amcl2 "${ROS_LIBS}")
  endif()
  target_compile_definitions(amcl2 PUBLIC USE_ASSERT_EXCEPTION USE_MAP_PUB)
  target_link_libraries(
    amcl2
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    fawkesbaseapp
    Laser360Interface
    Position3DInterface
    LocalizationInterface
    fawkes_amcl_pf
    fawkes_amcl_map
    fawkes_amcl_sensors
    fawkes_amcl_utils
    fawkestf)
else()
  plugin_disabled_message(amcl2)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
include(libs)

add_amcl_extra_libs()
