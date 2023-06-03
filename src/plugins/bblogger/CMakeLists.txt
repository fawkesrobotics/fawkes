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

set(PLUGIN_bblogger
    ON
    CACHE BOOL "Build bblogger plugin")

set(PLUGIN_bblogreplay
    ON
    CACHE BOOL "Build bblogreplay plugin")

add_subdirectory(console)
if(PLUGIN_bblogger)
  add_library(bblogger MODULE bblogger_plugin.cpp log_thread.cpp)

  target_link_libraries(
    bblogger
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    SwitchInterface)
  depend_on_cpp_version(bblogger 11)

else()
  plugin_disabled_message("bblogger")
endif()
if(PLUGIN_bblogreplay)
  add_library(bblogreplay MODULE bblogreplay_plugin.cpp logreplay_thread.cpp
                                 logreplay_bt_thread.cpp bblogfile.cpp)

  depend_on_cpp_version(bblogreplay 11)
  target_link_libraries(bblogreplay fawkescore fawkesutils fawkesaspects
                        fawkesinterface fawkesblackboard)

else()
  plugin_disabled_message("bblogreplay")
endif()
