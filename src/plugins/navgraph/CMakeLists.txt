set(PLUGIN_navgraph
    ON
    CACHE BOOL ON)

if(PLUGIN_navgraph)
  add_subdirectory(static-constraints)
  pkg_check_modules(visualization_msgs visualization_msgs)
  pkg_check_modules(fawkes_msgs fawkes_msgs)
  set(SOURCES navgraph_thread.cpp navgraph_plugin.cpp)
  if(ROS_FOUND AND visualization_msgs_FOUND)
    list(APPEND SOURCES visualization_thread.cpp)
  else()
    build_skipped_message("navgraph visualization-features"
                          "ros-visualization_msgs-devel")
  endif()
  if(ROS_FOUND AND fawkes_msgs_FOUND)
    list(APPEND SOURCES rospub_thread.cpp)
  else()
    build_skipped_message("navgraph fawkes_msgs" "fawkes_msgs")
  endif()

  add_library(navgraph MODULE ${SOURCES})
  depend_on_navgraph(navgraph)
  depend_on_tf(navgraph)

  target_link_libraries(
    navgraph
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    fawkesnavgraph
    NavigatorInterface
    NavPathInterface
    fawkestf
    fawkesnavgraphaspect)

  if(ROS_FOUND)
    depend_on_ros(navgraph)
    target_link_libraries(navgraph fawkesrosaspect)
    if(visualization_msgs_FOUND)
      target_compile_options(navgraph PUBLIC ${visualization_msgs_CFLAGS})
      target_link_libraries(navgraph ${visualization_msgs_LDFLAGS})
      target_compile_definitions(navgraph PUBLIC HAVE_VISUALIZATION)
    else()
      build_skipped_message("navgraph" "ros-visualization_msgs-devel")
    endif()
  else()
    build_skipped_message("navgraph" "ROS")
  endif()
else()
  plugin_disabled_message(navgraph)
endif()
