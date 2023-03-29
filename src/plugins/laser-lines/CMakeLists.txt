set(PLUGIN_laser
    -lines ON
    CACHE BOOL ON)
set(laser
    -
    lines_PCL_LIBS
    sample_consensus
    segmentation
    filters
    surface
    search)
check_deps_pcl_extra_libs("${laser-lines_PCL_LIBS}")

find_package(Boost COMPONENTS thread)
add_subdirectory(interfaces)

if(PLUGIN_laser-lines)
  if(Boost_THREAD_FOUND
     AND PCL_FOUND
     AND TF_FOUND
     AND NOT PCL_WARNING)
    add_library(
      laser
      -
      lines
      MODULE
      laser
      -
      lines
      -
      plugin.cpp
      laser
      -
      lines
      -
      thread.cpp
      line_info.cpp)
    target_link_libraries(laseliness ${ Boost_THREAD_LIBRARY})
    if(ROS_FOUND)
      target_link_libraries(laser-lines fawkesrosaspect)
    else()
      message(
        WARNING "Omitting ROS support laser-lines (ROS dependency missing)")
    endif()
    target_compile_definitions(
      laser
      -lines
      PUBLIC _FILE_OFFSET_BITS = 64 _LARGE_FILES)
    depend_on_tf(laser-lines)
    depend_on_pcl(laser-lines)
    depend_on_pcl_extra_libs(laser-lines "${laser-lines_PCL_LIBS}")
    target_link_libraries(
      laser
      -
      lines
      fawkescore
      fawkesutils
      fawkesaspects
      fvutils
      fawkesbaseapp
      fawkestf
      fawkesinterface
      fawkesblackboard
      fawkespcl_utils
      Position3DInterface
      SwitchInterface
      LaserLineInterface)
  else()
    message(WARNING "Omitting Plugin laser-cluster")
    if(NOT Boost_THREAD_FOUND)
      message(WARNING "Boost thread dependency missing")
    endif()
    if(NOT PCL_FOUND)
      message(WARNING "pcl[-devel] dependency missing")
    endif()
    if(NOT TF_FOUND)
      message(WARNING "bullet[-devel] dependency missing")
    endif()
    if(PCL_WARNING)
      foreach(pcl_warn ${PCL_WARNING})
        message(WARNING "${pcl_warn}")
      endforeach()
    endif()
  endif()
else()
  plugin_disabled_message(laser-lines)
endif()
