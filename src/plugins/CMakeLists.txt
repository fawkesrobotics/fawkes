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

set_output_directory_plugins()

set(CMAKE_SHARED_MODULE_PREFIX "")
include_directories(${PROJECT_SOURCE_DIR}/src)

add_subdirectory(amcl)
add_subdirectory(bblogger)
add_subdirectory(bbsync)
add_subdirectory(clips)
add_subdirectory(clips-agent)
add_subdirectory(clips-executive)
add_subdirectory(clips-gym)
add_subdirectory(clips-navgraph)
add_subdirectory(clips-pddl-parser)
add_subdirectory(clips-protobuf)
add_subdirectory(clips-robot-memory)
add_subdirectory(clips-tf)
add_subdirectory(execution-time-estimator)
add_subdirectory(execution-time-estimator-lookup)
add_subdirectory(execution-time-estimator-navgraph)
add_subdirectory(flite)
add_subdirectory(gazebo)
add_subdirectory(imu)
add_subdirectory(joystick)
add_subdirectory(laser)
add_subdirectory(laser-cluster)
add_subdirectory(laser-filter)
add_subdirectory(laser-lines)
add_subdirectory(laser-pointclouds)
add_subdirectory(mongodb)
add_subdirectory(mongodb_log)
add_subdirectory(navgraph)
add_subdirectory(navgraph-generator)
add_subdirectory(pddl-planner)
add_subdirectory(pddl-robot-memory)
add_subdirectory(perception)
add_subdirectory(realsense2)
add_subdirectory(robot-memory)
add_subdirectory(robotino)
add_subdirectory(ros)
add_subdirectory(ros2)
add_subdirectory(rrd)
add_subdirectory(skiller)
add_subdirectory(skiller-simulator)
add_subdirectory(static_transforms)
add_subdirectory(webview)
