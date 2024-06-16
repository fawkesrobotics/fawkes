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

set(YAMLCPP_DEPS "yaml-cpp")

# Function: depend_on_yamlcpp
# Usage: depend_on_yamlcpp(TARGET_NAME)
#
# This function adds the yamlcpp dependency to the target.
function(depend_on_yamlcpp target)
  target_compile_options(${target} PUBLIC -DHAVE_YAMLCPP ${yaml-cpp_CFLAGS})
  target_link_libraries(${target} ${yaml-cpp_LDFLAGS})
  target_compile_options(${target} PUBLIC -DHAVE_YAMLCPP_NODE_MARK)
endfunction()
