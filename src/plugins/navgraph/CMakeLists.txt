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

set(PLUGIN_navgraph
    ON
    CACHE BOOL ON)

add_subdirectory(static-constraints)
if(PLUGIN_navgraph)
  add_library(navgraph MODULE navgraph_thread.cpp navgraph_plugin.cpp)

  depend_on_navgraph(navgraph)
  depend_on_tf(navgraph)

  target_link_libraries(
    navgraph
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    fawkesnavgraph
    NavigatorInterface
    NavPathInterface
    fawkestf
    fawkesnavgraphaspect)

  if(ROS_1_FOUND)
    depend_on_ros(navgraph)
    optional_depend_on_ros_libs(navgraph visualization_msgs
                                visualization_msgs_dep_found)
    if(visualization_msgs_dep_found)
      pkg_check_modules(fawkes_msgs fawkes_msgs)
      target_sources(navgraph PRIVATE visualization_thread.cpp)
      target_compile_definitions(navgraph PUBLIC HAVE_VISUALIZATION)
    else()
      build_skipped_message("navgraph visualization-features"
                            "ros-visualization_msgs-devel")
    endif()
    optional_depend_on_ros_libs(navgraph fawkes_msgs fawkes_msgs_dep_found)
    if(fawkes_msgs_dep_found)
      target_sources(navgraph PRIVATE rospub_thread.cpp)
    else()
      build_skipped_message("navgraph fawkes_msgs" "fawkes_msgs")
    endif()
  else()
    build_skipped_message("navgraph" "ROS 1 features")
  endif()
else()
  plugin_disabled_message(navgraph)
endif()
