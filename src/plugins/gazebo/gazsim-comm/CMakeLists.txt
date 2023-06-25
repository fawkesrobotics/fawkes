# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann
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

set(PLUGIN_gazsim-comm
    ON
    CACHE BOOL "Build gazsim-comm plugin")

if(PLUGIN_gazsim-comm)
  add_library(gazsim-comm MODULE gazsim_comm_plugin.cpp gazsim_comm_thread.cpp)
  depend_on_boost_libs(gazsim-comm system)
  depend_on_protobuf_comm(gazsim-comm)
  target_link_libraries(gazsim-comm fawkescore fawkesaspects)
else()
  plugin_disabled_message("gazsim-comm")
endif()
