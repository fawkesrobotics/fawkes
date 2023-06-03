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

set(PLUGIN_rrd
    ON
    CACHE BOOL "Build rrd plugin")
add_subdirectory(aspect)

if(PLUGIN_rrd)
  add_library(rrd MODULE rrd_plugin.cpp rrd_thread.cpp)
  depend_on_pkgconfig_libs(rrd librrd)
  target_compile_definitions(rrd PUBLIC HAVE_RRD)
  target_link_libraries(rrd m fawkescore fawkesaspects fawkesutils
                        fawkesrrdaspect)
else()
  plugin_disabled_message("rrd")
endif()
