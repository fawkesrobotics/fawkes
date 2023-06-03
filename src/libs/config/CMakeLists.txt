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
  fawkesconfig SHARED
  memory.cpp
  change_handler.cpp
  # qa/qa_net_list_content.cpp
  # qa/qa_config_change_handler.cpp
  # qa/qa_yaml.cpp
  # qa/qa_config.cpp
  fawkesconfig_tolua.cpp
  net_list_content.cpp
  sqlite.cpp
  netconf.cpp
  net_handler.cpp
  config.cpp
  yaml.cpp)
if(EXISTS "${CMAKE_SYSROOT}/usr/include/sys/inotify.h")
  target_compile_options(fawkesconfig PUBLIC -DHAVE_INOTIFY)
endif()
depend_on_yamlcpp(fawkesconfig)

target_link_libraries(fawkesconfig stdc++ fawkescore fawkesutils fawkesnetcomm
                      fawkeslogging)

depend_on_pkgconfig_libs(fawkesconfig sqlite3)
target_compile_options(fawkesconfig PUBLIC -DHAVE_SQLITE)

set(LUA_SOURCES config.tolua)
set(LUA_LINK_LIBS fawkescore fawkesconfig)
generate_lua_lib(fawkesconfig "${LUA_SOURCES}" "${LUA_LINK_LIBS}")
