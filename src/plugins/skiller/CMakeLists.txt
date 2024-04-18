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

set(PLUGIN_skiller
    ON
    CACHE BOOL "Build skiller plugin")
set(SKILLER_TIMETRACKING
    OFF
    CACHE BOOL "set to enable time tracking for skiller")
add_subdirectory(interfaces)
add_subdirectory(rest-api)
add_subdirectory(consoletool)

if(PLUGIN_skiller)
  add_library(skiller MODULE skiller_feature.cpp exec_thread.cpp
                             skiller_plugin.cpp)

  target_link_libraries(
    skiller
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesnetcomm
    fawkeslua
    fawkesblackboard
    fawkesinterface
    fawkeslogging
    SkillerInterface
    SkillerDebugInterface)
else()
  plugin_disabled_message("skiller")
endif()
