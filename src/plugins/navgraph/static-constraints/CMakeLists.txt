set(PLUGIN_navgraph-static-constraints
    ON
    CACHE BOOL ON)

if(PLUGIN_navgraph-static-constraints)

  if(CPP_11_FOUND)
    add_library(navgraph-static-constraints MODULE navgraph_stconstr_plugin.cpp
                                                   navgraph_stconstr_thread.cpp)
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
    build_skipped_message("navgraph-static-constraints" "CPP11")
  endif()
else()
  plugin_disabled_message(navgraph-static-constraints)
endif()
