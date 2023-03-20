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
  message(STATUS "Skip building disabled amcl plugin")
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
  message(STATUS "Skip building disabled map-lasergern plugin")
endif()

block(SCOPE_FOR VARIABLES)
# Additional standalone libraries built from within this plugin
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_SHARED_MODULE_PREFIX "lib")
add_subdirectory(pf)
add_subdirectory(map)
add_subdirectory(sensors)
add_library(fawkes_amcl_utils SHARED amcl_utils.cpp)
target_link_libraries(fawkes_amcl_utils fawkescore fawkesconfig fvutils
                      fawkes_amcl_map)
endblock()
