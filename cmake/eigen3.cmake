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

set(EIGEN3_DEPS "eigen3")

# Function: depend_on_eigen3
# Usage: depend_on_eigen3(TARGET_NAME)
#
# Add the dependencies for eigen3 to the target.
function(depend_on_eigen3 target)
  depend_on_pkgconfig_libs(${target} eigen3)
  target_compile_options(
    ${target} PUBLIC -DHAVE_EIGEN3 -DEIGEN_USE_NEW_STDVECTOR
                     -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
  target_link_libraries(${target} tolua++)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${target} PRIVATE -Wno-deprecated-register)
  endif()
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${target} PRIVATE -Wno-deprecated-declarations)
  endif()
endfunction()
