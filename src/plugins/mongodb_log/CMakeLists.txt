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

set(PLUGIN_mongodb_log
    ON
    CACHE BOOL "Build mongodb_log plugin")

if(PLUGIN_mongodb_log)
  add_library(
    mongodb_log MODULE
    mongodb_log_bb_thread.cpp mongodb_log_image_thread.cpp
    mongodb_log_logger_thread.cpp mongodb_log_pcl_thread.cpp
    mongodb_log_plugin.cpp mongodb_log_tf_thread.cpp)
  depend_on_mongodb(mongodb_log)
  depend_on_tf(mongodb_log)
  depend_on_pcl(mongodb_log)

  target_link_libraries(
    mongodb_log
    fawkescore
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesutils
    fawkeslogging
    fawkesmongodbaspect
    fvutils
    fawkestf
    fawkespcl_utils)
  depend_on_cpp_version(mongodb_log 11)
else()
  plugin_disabled_message("mongodb_log")
endif()
