set(PLUGIN_amcl
    ON
    CACHE BOOL ON)
set(PLUGIN_map-lasergen
    ON
    CACHE BOOL ON)
if(PLUGIN_amcl)
  if(TF_FOUND)
    set(ROS_LIBS geometry_msgs nav_msgs)
    check_deps_extra_libs("${ROS_LIBS}")
    if(ROS_FOUND AND NOT EXTRA_LIBS_WARNING)
      add_library(amcl MODULE amcl_plugin.cpp amcl_thread.cpp ros_thread.cpp)
      depend_on_ros(amcl)
      depend_on_extra_libs(amcl "${ROS_LIBS}")
    else()
      add_library(amcl MODULE amcl_plugin.cpp amcl_thread.cpp)
      if(NOT ROS_FOUND)
        message(WARNING "Omitting amcl ROS interaction (ROS not found)")
      endif()
      foreach(warn EXTRA_LIBS_WARNING)
        message(WARNING ${warn})
      endforeach()
    endif()
    depend_on_tf(amcl)
    target_compile_definitions(amcl PUBLIC USE_ASSERT_EXCEPTION USE_MAP_PUB)
    target_link_libraries(
      amcl
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesinterface
      fawkesblackboard
      fawkesbaseapp
      Laser360Interface
      Position3DInterface
      LocalizationInterface
      fawkes_amcl_pf
      fawkes_amcl_map
      fawkes_amcl_sensors
      fawkes_amcl_utils
      fawkestf)

  else()
    message(WARNING "Omitting amcl plugin (bullet[-devel] dependency missing)")
  endif()
else()
  plugin_disabled_message(amcl)
endif()

if(PLUGIN_map-lasergen)
  if(TF_FOUND)
    add_library(map-lasergen MODULE map_lasergen_plugin.cpp
                                    map_lasergen_thread.cpp)
    depend_on_tf(map-lasergen)
    target_compile_definitions(map-lasergen PUBLIC USE_ASSERT_EXCEPTION
                                                   USE_MAP_PUB)
    target_link_libraries(
      map-lasergen
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesinterface
      fawkesblackboard
      fawkesbaseapp
      Laser360Interface
      Position3DInterface
      fawkes_amcl_map
      fawkes_amcl_utils
      fawkestf)
  else()
    message(
      WARNING "Omitting map-lasergen plugin (bullet[-devel] dependency missing)"
    )
  endif()
else()
  plugin_disabled_message(map-lasergen)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
include(libs)

add_amcl_extra_libs()
