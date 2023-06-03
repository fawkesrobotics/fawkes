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

set(PLUGIN_mongodb
    ON
    CACHE BOOL "Build mongodb plugin")

add_subdirectory(interfaces)
add_subdirectory(aspect)

set(PLUGIN_mongodb-rrd
    ON
    CACHE BOOL "Build mongodb-rrd plugin")

if(PLUGIN_mongodb)
  add_library(
    mongodb MODULE
    mongodb_plugin.cpp mongodb_thread.cpp mongodb_client_config.cpp
    mongodb_instance_config.cpp mongodb_replicaset_config.cpp utils.cpp)
  depend_on_mongodb(mongodb)
  depend_on_boost_libs(mongodb "system;filesystem")

  target_link_libraries(
    mongodb
    fawkescore
    fawkesaspects
    fawkesmongodbaspect
    fawkesblackboard
    MongoDBManagedReplicaSetInterface
    pthread
    ${Boost_LIBRARIES})
  target_compile_features(mongodb PUBLIC cxx_std_11)
  target_include_directories(mongodb PUBLIC ${Boost_INCLUDE_DIRS})
else()
  plugin_disabled_message("mongodb")
endif()

if(PLUGIN_mongodb-rrd)
  add_library(mongodb-rrd MODULE mongorrd_plugin.cpp mongorrd_thread.cpp)

  depend_on_mongodb(mongodb-rrd)

  target_link_libraries(
    mongodb-rrd
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesconfig
    fawkesmongodbaspect
    fawkesrrdaspect
    pthread
    ${Boost_LIBRARIES})
  target_compile_features(mongodb-rrd PUBLIC cxx_std_11)
  target_include_directories(mongodb-rrd PUBLIC ${Boost_LIBRARIES})
else()
  plugin_disabled_message("mongodb-rrd")
endif()
