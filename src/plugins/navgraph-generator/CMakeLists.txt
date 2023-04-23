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

set(PLUGIN_navgraph-generator
    ON
    CACHE BOOL ON)

add_subdirectory(interfaces)

if(PLUGIN_navgraph-generator)
  add_library(navgraph-generator MODULE navgraph_generator_thread.cpp
                                        navgraph_generator_plugin.cpp)
  depend_on_navgraph(navgraph-generator)
  depend_on_tf(navgraph-generator)
  depend_on_pcl(navgraph-generator)
  depend_on_cgal_libs(navgraph-generator)

  target_link_libraries(
    navgraph-generator
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    fawkesnavgraph
    fawkes_amcl_utils
    fawkes_amcl_map
    fawkesnavgraphaspect
    fawkesnavgraphgenerators
    NavGraphGeneratorInterface)

  depend_on_pcl_extra_libs(
    navgraph-generator "sample_consensus;segmentation;filters;surface;search")

  if(ROS_1_FOUND)
    depend_on_ros(navgraph-generator)
    build_depends_on(navgraph-generator fawkesrosaspect)
    target_link_libraries(navgraph-generator fawkesrosaspect)
    optional_depend_on_pkgconfig_libs(navgraph-generator visualization_msgs
                                      visualzation_msgs_deps_found)
    if(visualzation_msgs_deps_found)
      target_sources(navgraph-generator PRIVATE visualization_thread.cpp)
      target_compile_definitions(navgraph-generator PUBLIC HAVE_VISUALIZATION)
    else()
      build_skipped_message("navgraph-generator" "ros-visualization_msgs-devel")
    endif()
  else()
    build_skipped_message("navgraph-generator" "ROS 1 (roscpp)")
  endif()
else()
  plugin_disabled_message("navgraph-generator")
endif()
