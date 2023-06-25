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

set(PLUGIN_joystick
    ON
    CACHE BOOL "Build joystick plugin")

set(PLUGIN_joystick-teleop
    ON
    CACHE BOOL "Build joystick-teleop plugin")

set(EXECUTABLE_ffjoystick
    ON
    CACHE BOOL "Build ffjoystick tool")

set(EXECUTABLE_ffkbjoystick
    ON
    CACHE BOOL "Build ffkbjoystick tool")

if(PLUGIN_joystick)
  add_library(
    joystick MODULE
    joystick_plugin.cpp sensor_thread.cpp acquisition_thread.cpp bb_handler.cpp
    force_feedback.cpp act_thread.cpp)
  target_link_libraries(
    joystick
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    JoystickInterface)
else()
  plugin_disabled_message(joystick)
endif()

if(PLUGIN_joystick-teleop)
  add_library(joystick-teleop MODULE joystick_teleop_plugin.cpp
                                     joystick_teleop_thread.cpp)
  target_link_libraries(
    joystick-teleop
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface)
else()
  plugin_disabled_message(joystick-teleop)
endif()

if(EXECUTABLE_ffjoystick)
  add_executable(
    ffjoystick ffjoystick.cpp acquisition_thread.cpp remote_bb_poster.cpp
               bb_handler.cpp force_feedback.cpp act_thread.cpp)
  target_link_libraries(
    ffjoystick
    stdc++
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkeslogging
    JoystickInterface)
else()
  executable_disabled_message(ffjoystick)
endif()

if(EXECUTABLE_ffkbjoystick)
  add_executable(ffkbjoystick ffkbjoystick.cpp remote_bb_poster.cpp
                              bb_handler.cpp)
  target_link_libraries(
    ffkbjoystick
    stdc++
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkeslogging
    JoystickInterface)
else()
  executable_disabled_message(ffkbjoystick)
endif()
