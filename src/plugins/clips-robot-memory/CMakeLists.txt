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

set(PLUGIN_clips-robot-memory
    ON
    CACHE BOOL "Build clips-robot-memory plugin")

if(PLUGIN_clips-robot-memory)
  add_library(
    clips-robot-memory MODULE
    clips_robot_memory_plugin.cpp clips_robot_memory_thread.cpp
    clips_rm_trigger.cpp)
  depend_on_clipsmm(clips-robot-memory)
  depend_on_robot_memory(clips-robot-memory)
  target_link_libraries(
    clips-robot-memory
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    fawkesclipsaspect
    fawkesrobotmemory)
  target_compile_features(clips-robot-memory PUBLIC cxx_std_11)
else()
  plugin_disabled_message("clips-robot-memory")
endif()
