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

add_library(fawkes_clips_gym SHARED clips_gym_thread.cpp clips-observation-info.cpp goalAction.cpp)

target_include_directories(fawkes_clips_gym PRIVATE ${Python3_INCLUDE_DIRS})

target_link_libraries(
    fawkes_clips_gym
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

depend_on_clipsmm(fawkes_clips_gym)
depend_on_cpp_version(fawkes_clips_gym 17)