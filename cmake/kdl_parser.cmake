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

pkg_check_modules(urdfdom_headers_4 urdfdom_headers>=0.40)
remember_dependency(urdfdom_headers_4)
if(urdfdom_headers_4_FOUND)
  set(HAVE_URDFDOM_TYPES_H 1)
endif()

# Function: deppend_on_kdl_parser
# Usage: depend_on_kdl_parser(TARGET_NAME)
#
# Adds the necessary dependencies for kdl_parser to the target.
function(depend_on_kdl_parser target)
  optional_depend_on_pkgconfig_libs(${target} urdfdom_headers_4
                                    urdfdom_headers_4_deps_found)
  if(urdfdom_headers_4_deps_found)
    target_compile_options(${target} PUBLIC -DHAVE_URDFDOM_TYPES_H)
  else()
    depend_on_pkgconfig_libs(${target} urdfdom_headers)
  endif()
  optional_depend_on_pkgconfig_libs(${target} orocos-kdl orocos-kdl_deps_found)
  if(NOT orocos-kdl_deps_found)
    depend_on_pkgconfig_libs(${target} orocos_kdl)
  endif()
  depend_on_pkgconfig_libs(${target} "urdfdom;tinyxml")
endfunction()
