set(PLUGIN_clips-navgraph
    ON
    CACHE BOOL ON)

if(PLUGIN_clips-navgraph)
  if(CPP_11_FOUND)
    add_library(clips-navgraph MODULE clips_navgraph_plugin.cpp
                                      clips_navgraph_thread.cpp)
    depend_on_clipsmm(clips-navgraph)
    depend_on_navgraph(clips-navgraph)
    target_link_libraries(
      clips-navgraph
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesclipsaspect
      fawkesnavgraphaspect
      fawkesnavgraph)
  else()
    build_skipped_message("clips-navgraph" "CPP11")
  endif()
else()
  plugin_disabled_message(clips-navgraph)
endif()
