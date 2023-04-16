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

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${FAWKES_EXECUTABLE_DIR})
set(EXECUTABLE_fflaser_deadspots
    ON
    CACHE BOOL ON)

if(EXECUTABLE_fflaser_deadspots)
  add_executable(fflaser_deadspots deadspots.cpp)
  target_link_libraries(
    fflaser_deadspots
    stdc++
    m
    fawkescore
    fawkesutils
    fawkesnetcomm
    fawkesblackboard
    fawkesinterface
    fawkesconfig
    Laser360Interface
    Laser720Interface)
else()
  executable_disabled_message(fflaser_deadspots)
endif()
