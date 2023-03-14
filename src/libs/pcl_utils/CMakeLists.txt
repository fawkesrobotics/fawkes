if(TF_FOUND)
  if(PCL_FOUND)

    add_library(fawkespcl_utils SHARED pcl_adapter.cpp pointcloud_manager.cpp
                                       storage_adapter.cpp)

    target_link_libraries(fawkespcl_utils fawkescore fawkestf)
    depend_on_pcl(fawkespcl_utils)
  else()
    message(WARNING "Omitting PCL utility library (pcl[-devel] not installed)")
  endif()
else()
  message(WARNING "Omitting PCL utility library (tf framework not available)")
endif()
