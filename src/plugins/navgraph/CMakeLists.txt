set(PLUGIN_navgraph
    ON
    CACHE BOOL ON)

add_subdirectory(static-constraints)
if(PLUGIN_navgraph)
  add_library(navgraph MODULE navgraph_thread.cpp navgraph_plugin.cpp)

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
    optional_depend_on_ros_libs(navgraph visualization_msgs
                                visualization_msgs_dep_found)
    if(visualization_msgs_dep_found)
      pkg_check_modules(fawkes_msgs fawkes_msgs)
      target_sources(navgraph PRIVATE visualization_thread.cpp)
      target_compile_definitions(navgraph PUBLIC HAVE_VISUALIZATION)
    else()
      build_skipped_message("navgraph visualization-features"
                            "ros-visualization_msgs-devel")
    endif()
    optional_depend_on_ros_libs(navgraph fawkes_msgs fawkes_msgs_dep_found)
    if(fawkes_msgs_dep_found)
      target_sources(navgraph PRIVATE rospub_thread.cpp)
    else()
      build_skipped_message("navgraph fawkes_msgs" "fawkes_msgs")
    endif()
  else()
    build_skipped_message("navgraph" "ROS 1 features")
  endif()
else()
  plugin_disabled_message(navgraph)
endif()
