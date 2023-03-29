add_library(fawkesrrdaspect SHARED rrd.cpp rrd_descriptions.cpp rrd_inifin.cpp)

set_target_properties(fawkesrrdaspect PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                                 ${FAWKES_LIB_DIR})
target_link_libraries(fawkesrrdaspect stdc++ fawkescore fawkesaspects
                      fawkesutils)
