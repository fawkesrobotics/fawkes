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

set(PLUGIN_clips-gym
    ON
    CACHE BOOL "Build clips-gym")
message(
  "------------------------------------- Pyhton interpreter found: ${Python3_INCLUDE_DIRS}  ${Python3_LIBRARIES}"
)
if(PLUGIN_clips-gym)
  add_library(clips-gym-thread SHARED clips-gym-thread.cpp
                                      clips-observation-info.cpp goalAction.cpp)
  depend_on_clipsmm(clips-gym-thread)
  depend_on_cpp_version(clips-gym-thread 17)
  target_include_directories(clips-gym-thread PRIVATE ${Python3_INCLUDE_DIRS})

  # required for pybind11 set_target_properties(clips-gym-thread PROPERTIES
  # CXX_VISIBILITY_PRESET hidden) target_link_options(clips-gym-thread PRIVATE
  # ${Python3_LINK_OPTIONS})
  target_link_libraries(
    clips-gym-thread
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    fawkesclipsaspect
    fawkeslogging
    ${Python3_LIBRARIES}
    Python3::Python
    pybind11::headers)
  add_library(clips-gym MODULE clips-gym-plugin.cpp)

  depend_on_clipsmm(clips-gym)
  depend_on_cpp_version(clips-gym 17)
  target_include_directories(clips-gym PRIVATE ${Python3_INCLUDE_DIRS})

  # required for pybind11 set_target_properties(clips-gym PROPERTIES
  # CXX_VISIBILITY_PRESET hidden) target_link_options(clips-gym PRIVATE
  # ${Python3_LINK_OPTIONS})
  target_link_libraries(clips-gym clips-gym-thread)
else()
  plugin_disabled_message("clips-gym")
endif()
