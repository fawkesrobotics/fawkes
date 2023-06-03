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

set(PLUGIN_robot-memory
    ON
    CACHE BOOL "Build robot-memory plugin")

add_subdirectory(interfaces)
add_subdirectory(aspect)

if(PLUGIN_robot-memory)
  add_library(
    robot-memory MODULE
    robot_memory.cpp
    computables/blackboard_computable.cpp
    computables/transform_computable.cpp
    computables/computable.cpp
    computables/computables_manager.cpp
    event_trigger.cpp
    robot_memory_thread.cpp
    event_trigger_manager.cpp
    robot_memory_plugin.cpp)
  depend_on_robot_memory(robot-memory)

  target_link_libraries(
    robot-memory
    fawkescore
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesutils
    fawkeslogging
    fawkesmongodbaspect
    fvutils
    fawkestf
    RobotMemoryInterface
    fawkesrobotmemory)
  target_compile_features(robot-memory PUBLIC cxx_std_17)
else()
  plugin_disabled_message("robot-memory")
endif()
