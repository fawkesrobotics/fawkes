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

set(PLUGIN_navgraph-static-constraints
    ON
    CACHE BOOL ON)

if(PLUGIN_navgraph-static-constraints)

  add_library(navgraph-static-constraints MODULE navgraph_stconstr_plugin.cpp
                                                 navgraph_stconstr_thread.cpp)
  target_compile_features(navgraph-static-constraints PUBLIC cxx_std_11)
  depend_on_tf(navgraph-static-constraints)
  depend_on_ros(navgraph-static-constraints)
  target_link_libraries(
    navgraph-static-constraints
    fawkescore
    fawkesutils
    fawkesnavgraph
    fawkesaspects
    fawkesnavgraphaspect
    fawkestf
    fawkesnavgraphaspect)
else()
  plugin_disabled_message(navgraph-static-constraints)
endif()
