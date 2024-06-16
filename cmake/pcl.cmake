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

include(ros)
include(eigen3)

set(PCL_SUFFIX "")
set(PCL_COMMON "pcl_common")
pkg_check_modules(PCL pcl_common)
remember_dependency(PCL)

if(NOT PCL_FOUND)
  message("Check for plc_common with explicit version suffix")
  execute_process(
    COMMAND pkg-config --list-all
    COMMAND grep pcl_common
    COMMAND awk "{ print $1 }"
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE PCL_COMMON)
  pkg_check_modules(PCL ${PCL_COMMON})
  string(REGEX REPLACE "^pcl_common" "" PCL_SUFFIX ${PCL_COMMON})
endif()
if(EXISTS "${CMAKE_SYSROOT}/usr/include/vtk/vtkVersion.h")
  set(CFLAGS_VTK -I/usr/include/vtk)
endif()

# Function to depend on PCL
# usage: depend_on_pcl(target)
#
# Adds the necessary include directories and libraries to the target
function(depend_on_pcl target)
  depend_on_pkgconfig_libs(${target} ${PCL_COMMON})
  depend_on_eigen3(${target})
  target_compile_options(${target} PUBLIC -DHAVE_PCL)
  target_compile_options(
    ${target} PRIVATE -Wno-unknown-pragmas -Wno-deprecated-declarations
                      -Wno-overloaded-virtual -Wno-array-bounds)
endfunction()

# Function to depend on PCL extra libs
function(depend_on_pcl_extra_libs target libs)
  set(pcl_libs)
  foreach(pcl_lib ${libs})
    list(APPEND pcl_libs pcl_${pcl_lib}${PCL_SUFFIX})
  endforeach()
  depend_on_pkgconfig_libs(${target} "${pcl_libs}")
endfunction()

# TODO link against ros std_msgs!?
