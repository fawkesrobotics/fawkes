add_library(fawkesrobotmemory SHARED robot_memory_inifin.cpp
                                     robot_memory_aspect.cpp)

depend_on_mongodb(fawkesrobotmemory)
target_compile_features(fawkesrobotmemory PUBLIC cxx_std_17)
set_target_properties(fawkesrobotmemory PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                                   ${FAWKES_LIB_DIR})
