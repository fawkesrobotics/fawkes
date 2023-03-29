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
  set(SOURCES skiller_feature.cpp exec_thread.cpp skiller_plugin.cpp)

  set(NAVGRAPH_FOUND)
  check_navgraph_deps(NAVGRAPH_FOUND)
  if(NAVGRAPH_FOUND)
    list(APPEND SOURCES skiller_navgraph_feature.cpp)
  else()
    message(INFO "Skipping skiller navgraph feature")
  endif()
  add_library(skiller MODULE ${SOURCES})
  if(NAVGRAPH_FOUND)
    target_link_libraries(skiller fawkesnavgraphaspect)
    target_compile_definitions(skiller PUBLIC HAVE_NAVGRAPH)
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
