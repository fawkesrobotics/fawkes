add_library(fawkespcl_utils SHARED pcl_adapter.cpp pointcloud_manager.cpp
                                   storage_adapter.cpp)
build_depends_on(fawkespcl_utils fawkestf)
target_link_libraries(fawkespcl_utils fawkescore fawkestf)
depend_on_pcl(fawkespcl_utils)
