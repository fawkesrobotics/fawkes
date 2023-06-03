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

set(PLUGIN_skiller-simulator
    ON
    CACHE BOOL "Build skiller-simulator plugin")

if(PLUGIN_skiller-simulator)
  if(CPP_17_FOUND)
    add_library(skiller-simulator MODULE exec_thread.cpp
                                         skiller_simulator_plugin.cpp)

    target_link_libraries(
      skiller-simulator
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkeslogging
      fawkes_execution_time_estimator
      fawkes_execution_time_estimator_aspect)
  else()
    build_skipped_message("skiller-simulator" "CPP17")
  endif()
else()
  plugin_disabled_message("skiller-simulator")
endif()
