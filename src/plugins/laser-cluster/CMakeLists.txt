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

set(PLUGIN_laser-cluster
    ON
    CACHE BOOL "Build laser-cluster plugin")
set(LASER_CLUSTER_PCL_LIBS sample_consensus segmentation filters surface search)

if(PLUGIN_laser-cluster)
  add_subdirectory(interfaces)
  add_library(laser-cluster MODULE laser-cluster-plugin.cpp
                                   laser-cluster-thread.cpp)
  depend_on_pcl(laser-cluster)
  depend_on_tf(laser-cluster)
  depend_on_pcl_extra_libs(laser-cluster "${LASER_CLUSTER_PCL_LIBS}")
  target_compile_features(laser-cluster PUBLIC cxx_std_11)
  target_link_libraries(
    laser-cluster
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
    LaserClusterInterface)
  depend_on_boost_libs(laser-cluster thread)
  target_compile_options(laser-cluster PUBLIC -Wno-deprecated)
  target_compile_definitions(laser-cluster PUBLIC _FILE_OFFSET_BITS=64
                                                  _LARGE_FILES)
else()
  plugin_disabled_message(laser-cluster)
endif()
