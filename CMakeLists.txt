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

if(NOT FAWKES_DIR)
  cmake_minimum_required(VERSION 3.5)
  project(fawkes)
  set(_FAWKES_DEPENDENCIES_CHECKED
      ""
      CACHE INTERNAL "")
  # make local cmake modules available
  set(FAWKES_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${FAWKES_CORE_DIR}/cmake")
  include(base)
endif()

add_subdirectory(src)
add_subdirectory(etc)

if(NOT FAWKES_DIR)
  set_common_properties_of_targets(all_targets)
endif()
# add_subdirectory(src/libs) add_subdirectory(src/plugins)
# add_subdirectory(src/mainapp) add_subdirectory(src/tools)
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# add_compile_options(-Wall -Wextra -Wpedantic) endif()
#
# collect the files for subdir1 include(src/libs//CMakeLists.txt) foreach(FILE
# ${FILES}) set(subdir1Files ${subdir1Files} subdir1/${FILE}) endforeach()
#
# collect the files for subdir2 include(subdir2/CMakeLists.txt) foreach(FILE
# ${FILES}) set(subdir2Files ${subdir2Files} subdir2/${FILE}) endforeach()
#
# add the source files to the executable add_executable(fawkes)
#
# ${subdir1Files} ${subdir2Files
