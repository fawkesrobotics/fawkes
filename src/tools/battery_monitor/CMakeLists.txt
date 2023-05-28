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

set(EXECUTABLE_ffbatterymon
    ON
    CACHE BOOL ON)

if(EXECUTABLE_ffbatterymon)
  add_executable(ffbatterymon main.cpp battery_monitor_treeview.cpp
                              battery_monitor.cpp)
  depend_on_pkgconfig_libs(ffbatterymon "ncurses;readline")
  depend_on_gui(ffbatterymon)
  target_link_libraries(
    ffbatterymon
    stdc++
    fawkescore
    fawkesconfig
    fawkesnetcomm
    fawkesgui_utils
    fawkesblackboard
    fawkesinterface
    BatteryInterface)
else()
  executable_disabled_message(ffbatterymon)
endif()
