# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann
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
find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)

set(PLUGIN_clips-gym
    ON
    CACHE BOOL "Build clips-gym")

if(PLUGIN_clips-gym)
  add_library(clips-gym MODULE clips-gym-plugin.cpp clips-gym-thread.cpp
                               clips-observation-info.cpp goalAction.cpp)

  depend_on_clipsmm(clips-gym)
  depend_on_cpp_version(clips-gym 17)
  target_link_libraries(clips-gym pybind11::headers)

  # required for pybind11
  set_target_properties(clips-gym PROPERTIES CXX_VISIBILITY_PRESET hidden)

  target_link_libraries(
    clips-gym
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    fawkesclipsaspect
    fawkeslogging
    Python3::Module)
else()
  plugin_disabled_message("clips-gym")
endif()
