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

# Function: add_amcl_extra_libs
# Usage: add_amcl_extra_libs
#
# A function to add the extra depedencies for the amcl only needed in amcl.cmake
function(add_amcl_extra_libs)
  # Additional standalone libraries built from within this plugin

  # cmake-lint: disable=C0103
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_SHARED_MODULE_PREFIX "lib")
  add_subdirectory(pf)
  add_subdirectory(map)
  add_subdirectory(sensors)
  add_library(fawkes_amcl_utils SHARED amcl_utils.cpp)
  target_link_libraries(fawkes_amcl_utils fawkescore fawkesconfig fvutils
                        fawkes_amcl_map)
endfunction()
