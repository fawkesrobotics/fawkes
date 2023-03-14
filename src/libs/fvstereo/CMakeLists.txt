add_library(fvstereo SHARED stereo_processor.cpp)

depend_on_fvconf(fvstereo)
target_link_libraries(fvstereo fawkescore fawkesutils fvutils fvcams)
