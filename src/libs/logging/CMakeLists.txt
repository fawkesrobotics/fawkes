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

include_directories(libs)
add_library(
  fawkeslogging SHARED
  liblogger.cpp
  factory.cpp
  console.cpp
  cache.cpp
  file.cpp
  multi.cpp
  syslog.cpp
  component.cpp
  logger.cpp
  fd_redirect.cpp)

depend_on_boost_libs(fawkeslogging system)

generate_tolua(fawkeslogging component.tolua)
set(LUA_SOURCES component.tolua)
generate_lua_lib(fawkeslogging "${LUA_SOURCES}" "")

target_link_libraries(fawkeslogging pthread stdc++ fawkescore
                      ${Boost_SYSTEM_LIBRARY})
target_include_directories(fawkesutils PRIVATE logging)
