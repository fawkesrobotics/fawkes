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

pkg_check_modules(libmongocxx libmongocxx>3)
remember_dependency(libmongocxx)

function(depend_on_mongodb target)
  depend_on_tf(${target})
  depend_on_pkgconfig_libs(${target} libmongocxx)
  target_compile_options(${target} PUBLIC -Wno-deprecated)
endfunction()

function(depend_on_robot_memory target)
  depend_on_mongodb(${target})
endfunction()