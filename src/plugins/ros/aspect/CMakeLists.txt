add_library(fawkesrosaspect SHARED ros.cpp ros_inifin.cpp)
set_target_properties(fawkesrosaspect PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                                 ${FAWKES_LIB_DIR})

target_link_libraries(fawkesrosaspect stdc++ fawkescore fawkesaspects
                      fawkesutils)
depend_on_ros(fawkesrosaspect)
