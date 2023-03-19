set(PLUGIN_laser-cluster
    ON
    CACHE BOOL ON)
set(laser-cluster_PCL_LIBS sample_consensus segmentation filters surface search)
check_deps_pcl_extra_libs("${laser-cluster_PCL_LIBS}")

find_package(Boost COMPONENTS thread)

if(PLUGIN_laser-cluster)
  if(Boost_THREAD_FOUND
     AND PCL_FOUND
     AND TF_FOUND
     AND NOT PCL_WARNING)
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
    target_link_libraries(laser-cluster ${Boost_THREAD_LIBRARY})
    target_compile_options(laser-cluster PUBLIC -Wno-deprecated)
    target_compile_definitions(laser-cluster PUBLIC _FILE_OFFSET_BITS=64
                                                    _LARGE_FILES)
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
  message(STATUS "Skip building disabled laser-cluster plugin")
endif()
