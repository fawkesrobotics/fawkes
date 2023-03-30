set(PLUGIN_navgraph-generator
    ON
    CACHE BOOL ON)

add_subdirectory(interfaces)

if(PLUGIN_navgraph-generator)
  if(CPP_11_FOUND)
    pkg_check_modules(visualization_msgs visualization_msgs)
    pkg_check_modules(fawkes_msgs fawkes_msgs)
    set(SOURCES navgraph_generator_thread.cpp navgraph_generator_plugin.cpp)
    if(ROS_FOUND AND visualization_msgs_FOUND)
      list(APPEND SOURCES visualization_thread.cpp)
    else()
      build_skipped_message("navgraph-generator visualization-features"
                            "ros-visualization_msgs-devel")
    endif()
    check_deps_pcl_extra_libs(navgraph-generator sample_consensus segmentation
                              filters surface search)
    if(NOT PCL_WARNING)
      add_library(navgraph MODULE ${SOURCES})
      depend_on_navgraph(navgraph-generator)
      depend_on_tf(navgraph-generator)
      depend_on_pcl(navgraph-generator)
      depend_on_cgal(navgraph-generator)

      target_link_libraries(
        navgraph-generator
        fawkescore
        fawkesutils
        fawkesaspects
        fawkesinterface
        fawkesblackboard
        fawkesnavgraph
        fawkes_amcl_utils
        fawkes_amcl_map
        fakwesnavgraphaspect
        fawkesnavgraphgenerators
        NavGraphGenreatorInterface)

      depend_on_pcl_extra_libs(navgraph-generator sample_consensus segmentation
                               filters surface search)

      if(ROS_FOUND)
        depend_on_ros(navgraph-generator)
        target_link_libraries(navgraph-generator fawkesrosaspect)
        if(visualization_msgs_FOUND)
          target_compile_options(navgraph-generator
                                 PUBLIC ${visualization_msgs_CFLAGS})
          target_link_libraries(navgraph-generator
                                ${visualization_msgs_LDFLAGS})
          target_compile_definitions(navgraph-generator
                                     PUBLIC HAVE_VISUALIZATION)
        else()
          build_skipped_message("navgraph-generator"
                                "ros-visualization_msgs-devel")
        endif()
      else()
        build_skipped_message("navgraph-generator" "ROS")
      endif()
    else()
      foreach(pcl_warn ${PCL_WARNING})
        message(WARNING "${pcl_warn}")
      endforeach()
    endif()
  else()
    build_skipped_message("navgraph-generator" "CPP11")
  endif()
else()
  plugin_disabled_message("navgraph-generator")
endif()
