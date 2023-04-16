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

set(PLUGIN_laser-filter
    ON
    CACHE BOOL ON)
add_subdirectory(deadspots)
add_subdirectory(interfaces)

if(PLUGIN_laser-filter)

  add_library(
    laser-filter MODULE
    filter_thread.cpp
    filters/deadspots.cpp
    filters/cascade.cpp
    filters/circle_sector.cpp
    filters/min_merge.cpp
    filters/reverse_angle.cpp
    filters/min_circle.cpp
    filters/filter.cpp
    filters/map_filter.cpp
    filters/720to360.cpp
    filters/1080to360.cpp
    filters/box_filter.cpp
    filters/copy.cpp
    filters/max_circle.cpp
    laser_filter_plugin.cpp)
  optional_depend_on_pkgconfig_libs(laser-filter "${tf_deps}" tf_deps_found)
  if(tf_deps_found)
    target_sources(laser-filter PRIVATE filters/projection.cpp)
    depend_on_tf(laser-filter)
  else()
    build_skipped_message("projection filter of laser-filter plugin"
                          "${tf_deps}")
  endif()
  target_link_libraries(
    laser-filter
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkes_amcl_utils
    fawkes_amcl_map
    Laser360Interface
    Laser720Interface
    Laser1080Interface
    LaserBoxFilterInterface)

else()
  plugin_disabled_message(laser-filter)
endif()
