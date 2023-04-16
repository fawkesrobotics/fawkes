add_library(fawkesmongodbaspect SHARED mongodb_inifin.cpp mongodb.cpp)

depend_on_mongodb(fawkesmongodbaspect)
target_compile_features(fawkesmongodbaspect PUBLIC cxx_std_11)
set_target_properties(fawkesmongodbaspect PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                                     ${FAWKES_LIB_DIR})
target_link_libraries(fawkesmongodbaspect fawkescore fawkesaspects)
