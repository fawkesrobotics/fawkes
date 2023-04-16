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
add_subdirectory(laser)
add_subdirectory(imu)
add_subdirectory(flite)
# add_subdirectory(festival)
add_subdirectory(joystick)
# add_subdirectory(openrave) add_subdirectory(katana) add_subdirectory(jaco)
# add_subdirectory(pantilt) add_subdirectory(roomba) add_subdirectory(nao)
add_subdirectory(robotino)
# add_subdirectory(bumblebee2) add_subdirectory(realsense)
add_subdirectory(bbsync)
add_subdirectory(realsense2)
add_subdirectory(perception)
add_subdirectory(amcl)
add_subdirectory(skiller)
add_subdirectory(skiller-simulator)
# add_subdirectory(execution-time-estimator-navgraph)
# add_subdirectory(execution-time-estimator-lookup luaagent)
add_subdirectory(laser-filter)
add_subdirectory(laser-lines)
add_subdirectory(laser-cluster)
add_subdirectory(laser-pointclouds)
add_subdirectory(static_transforms)
add_subdirectory(navgraph)
# navgraph-clusters)
add_subdirectory(navgraph-generator)
add_subdirectory(webview)
# add_subdirectory(colli)
add_subdirectory(clips)
add_subdirectory(clips-protobuf)
add_subdirectory(clips-navgraph)
add_subdirectory(clips-pddl-parser)
# add_subdirectory(clips-ros)
add_subdirectory(clips-tf)
# add_subdirectory(openprs) add_subdirectory(openprs-agent)
# add_subdirectory(eclipse-clp)
add_subdirectory(mongodb)
# add_subdirectory(mongodb_log)
add_subdirectory(robot-memory)
add_subdirectory(rrd)
add_subdirectory(clips-robot-memory)
add_subdirectory(pddl-robot-memory)
add_subdirectory(pddl-planner)
# add_subdirectory(openrave-robot-memory) add_subdirectory(openni)
# add_subdirectory(refboxcomm)
add_subdirectory(ros)
# add_subdirectory(player) add_subdirectory(xmlrpc) add_subdirectory(gossip)
# add_subdirectory(robot_state_publisher) add_subdirectory(gazebo)
# add_subdirectory(dynamixel) add_subdirectory(navgraph-interactive)
# add_subdirectory(stn-generator)
add_subdirectory(clips-executive)
# add_subdirectory(asp) add_subdirectory(plexil) add_subdirectory(cedar)
# add_subdirectory(hardware-models) add_subdirectory(execution-time-estimator)
