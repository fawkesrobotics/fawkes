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

set(PLUGIN_execution-time-estimator-lookup
    ON
    CACHE BOOL "Build execution-time-estimator-lookup plugin")

if(PLUGIN_execution-time-estimator-lookup)
  add_library(execution-time-estimator-lookup MODULE
              lookup_plugin.cpp lookup_thread.cpp lookup_estimator.cpp)
  depend_on_cpp_version(execution-time-estimator-lookup 17)
  depend_on_mongodb(execution-time-estimator-lookup)
  target_link_libraries(
    execution-time-estimator-lookup
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesmongodbaspect
    fawkes_execution_time_estimator
    fawkes_execution_time_estimator_aspect)
else()
  plugin_disabled_message("execution-time-estimator-lookup")
endif()
