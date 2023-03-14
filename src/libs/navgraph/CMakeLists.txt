set(NAVGRAPH_FOUND 0)
check_navgraph_deps(NAVGRAPH_FOUND)

if(NAVGRAPH_FOUND)
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
  if(LUA_FOUND)
    generate_tolua(fawkesnavgraph_tolua navgraph_node.tolua navgraph.tolua)
    add_library(fawkesnavgraph_tolua SHARED fawkesnavgraph_tolua.cpp)
    depend_on_lua(fawkesnavgraph_tolua)
    target_link_libraries(fawkesnavgraph_tolua fawkescore fawkesutils
                          fawkesnavgraph)
  endif()

else()
  message(WARNING "Skipped due to missing dependencies")
endif()
