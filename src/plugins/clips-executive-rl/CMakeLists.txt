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

add_subdirectory(interfaces)
set(PLUGIN_clips-executive-rl
    ON
    CACHE BOOL "Build clips-executive-rl")

if(PLUGIN_clips-executive-rl)
  add_library(clips-executive-rl MODULE rl-test-plugin.cpp rl-test-thread.cpp
                                        py-guard.cpp)
  depend_on_cpp_version(clips-executive-rl 17)
  target_link_libraries(clips-executive-rl pybind11::headers)
  depend_on_clipsmm(clips-executive-rl)

  # required for pybind11
  set_target_properties(clips-executive-rl PROPERTIES CXX_VISIBILITY_PRESET
                                                      hidden)

  # TODO: rlblocksworld includes?

  target_link_libraries(
    clips-executive-rl
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesclipsaspect
    fawkeslogging
    RLAgentGoalSelectionInterface
    Python3::Module)
else()
  plugin_disabled_message("clips-executive-rl")
endif()
