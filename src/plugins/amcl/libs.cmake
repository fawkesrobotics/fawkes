function(add_amcl_extra_libs)
  # Additional standalone libraries built from within this plugin
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_SHARED_MODULE_PREFIX "lib")
  add_subdirectory(pf)
  add_subdirectory(map)
  add_subdirectory(sensors)
  add_library(fawkes_amcl_utils SHARED amcl_utils.cpp)
  target_link_libraries(fawkes_amcl_utils fawkescore fawkesconfig fvutils
                        fawkes_amcl_map)
endfunction()
