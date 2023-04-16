set(PLUGIN_laser-cluster
    ON
    CACHE BOOL ON)
set(laser-cluster_PCL_LIBS sample_consensus segmentation filters surface search)

if(PLUGIN_laser-cluster)
  add_subdirectory(interfaces)
  add_library(laser-cluster MODULE laser-cluster-plugin.cpp
                                   laser-cluster-thread.cpp)
  depend_on_pcl(laser-cluster)
  depend_on_tf(laser-cluster)
  depend_on_pcl_extra_libs(laser-cluster "${laser-cluster_PCL_LIBS}")
  target_compile_features(laser-cluster PUBLIC cxx_std_11)
  target_link_libraries(
    laser-cluster
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
    LaserClusterInterface)
  depend_on_boost_libs(laser-cluster thread)
  target_compile_options(laser-cluster PUBLIC -Wno-deprecated)
  target_compile_definitions(laser-cluster PUBLIC _FILE_OFFSET_BITS=64
                                                  _LARGE_FILES)
else()
  plugin_disabled_message(laser-cluster)
endif()
