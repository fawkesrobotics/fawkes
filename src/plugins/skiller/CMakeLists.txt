set(PLUGIN_skiller
    ON
    CACHE BOOL ON)
set(SKILLER_TIMETRACKING
    OFF
    CACHE BOOL "set to enable time tracking for skiller")
add_subdirectory(interfaces)
add_subdirectory(rest-api)
add_subdirectory(consoletool)

if(PLUGIN_skiller)
  add_library(skiller MODULE skiller_feature.cpp exec_thread.cpp
                             skiller_plugin.cpp)

  optional_depend_on_pkgconfig_libs(skiller "${navgraph_deps}"
                                    navgraph_deps_found)
  if(navgraph_deps_found)
    target_sources(skiller PRIVATE skiller_navgraph_feature.cpp)
    target_link_libraries(skiller fawkesnavgraphaspect)
    target_compile_definitions(skiller PUBLIC HAVE_NAVGRAPH)
  else()
    build_skipped_message("skiller navgraph feature" "navgraph dependencies")
  endif()
  target_link_libraries(
    skiller
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesnetcomm
    fawkeslua
    fawkesblackboard
    fawkesinterface
    fawkeslogging
    SkillerInterface
    SkillerDebugInterface)
else()
  plugin_disabled_message("skiller")
endif()
