set(PLUGIN_laser-lines
    ON
    CACHE BOOL ON)
set(laser-lines_PCL_LIBS sample_consensus segmentation filters surface search)

add_subdirectory(interfaces)

if(PLUGIN_laser-lines)
  add_library(laser-lines MODULE laser-lines-plugin.cpp laser-lines-thread.cpp
                                 line_info.cpp)
  depend_on_boost_libs(laser-lines thread)
  if(ROS_1_FOUND)
    build_depends_on(laser-lines fawkesrosaspect)
    target_link_libraries(laser-lines fawkesrosaspect)
  else()
    build_skipped_message("laser-lines ROS support" "fawkesrosaspect")
  endif()
  target_compile_definitions(laser-lines PUBLIC _FILE_OFFSET_BITS=64
                                                _LARGE_FILES)
  depend_on_tf(laser-lines)
  depend_on_pcl(laser-lines)
  depend_on_pcl_extra_libs(laser-lines "${laser-lines_PCL_LIBS}")
  target_link_libraries(
    laser-lines
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
  plugin_disabled_message(laser-lines)
endif()
