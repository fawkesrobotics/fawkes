set(PLUGIN_laser-pointclouds
    ON
    CACHE BOOL ON)

if(PLUGIN_laser-pointclouds)

  add_library(laser-pointclouds MODULE laser_pointcloud_plugin.cpp
                                       laser_pointcloud_thread.cpp)

  depend_on_tf(laser-pointclouds)
  depend_on_pcl(laser-pointclouds)

  target_link_libraries(
    laser-pointclouds
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkestf
    fawkespcl_utils
    fawkesinterface
    Laser360Interface
    Laser720Interface
    Laser1080Interface)

else()
  plugin_disabled_message(laser-pointclouds)
endif()
