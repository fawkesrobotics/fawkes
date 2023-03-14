add_library(fawkesnavgraphaspect SHARED navgraph_inifin.cpp navgraph.cpp)
target_link_libraries(fawkesnavgraphaspect fawkescore fawkesaspects fawkesutils
                      fawkesnavgraph)
target_compile_features(fawkesnavgraphaspect PUBLIC cxx_std_11)
