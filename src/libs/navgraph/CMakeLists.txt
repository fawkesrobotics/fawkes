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

add_subdirectory(aspect)
add_subdirectory(generators)
add_library(
  fawkesnavgraph SHARED
  yaml_navgraph.cpp
  navgraph.cpp
  constraints/timed_reservation_list_node_constraint.cpp
  constraints/constraint_repo.cpp
  constraints/polygon_node_constraint.cpp
  constraints/static_list_edge_constraint.cpp
  constraints/timed_reservation_list_edge_constraint.cpp
  constraints/static_list_edge_cost_constraint.cpp
  constraints/edge_constraint.cpp
  constraints/polygon_constraint.cpp
  constraints/polygon_edge_constraint.cpp
  constraints/node_constraint.cpp
  constraints/edge_cost_constraint.cpp
  constraints/static_list_node_constraint.cpp
  navgraph_node.cpp
  search_state.cpp
  navgraph_path.cpp
  navgraph_edge.cpp)
depend_on_navgraph(fawkesnavgraph)
target_link_libraries(fawkesnavgraph stdc++ m fawkescore fawkesutils)

set(LUA_SOURCES navgraph_node.tolua navgraph.tolua)
set(LUA_LINK_LIBS fawkescore fawkesutils fawkesnavgraph)
generate_lua_lib(fawkesnavgraph "${LUA_SOURCES}" "${LUA_LINK_LIBS}")
