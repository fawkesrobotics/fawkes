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

set(PLUGIN_laser-lines
    ON
    CACHE BOOL "Build laser-lines plugin")
set(LASER-LINES_PCL_LIBS sample_consensus segmentation filters surface search)

add_subdirectory(interfaces)

if(PLUGIN_laser-lines)
  add_library(laser-lines MODULE laser-lines-plugin.cpp laser-lines-thread.cpp
                                 line_info.cpp)
  depend_on_boost_libs(laser-lines thread)
  if(ROS_1_FOUND)
    build_depends_on(laser-lines fawkesrosaspect)
    target_link_libraries(laser-lines fawkesrosaspect)
  else()
    build_skipped_message("laser-lines ROS support" "fawkesrosaspect")
  endif()
  target_compile_definitions(laser-lines PUBLIC _FILE_OFFSET_BITS=64
                                                _LARGE_FILES)
  depend_on_tf(laser-lines)
  depend_on_pcl(laser-lines)
  depend_on_pcl_extra_libs(laser-lines "${LASER-LINES_PCL_LIBS}")
  target_link_libraries(
    laser-lines
    fawkescore
    fawkesutils
    fawkesaspects
    fvutils
    fawkesbaseapp
    fawkestf
    fawkesinterface
    fawkesblackboard
    fawkespcl_utils
    Position3DInterface
    SwitchInterface
    LaserLineInterface)
else()
  plugin_disabled_message(laser-lines)
endif()
