add_library(fawkesnavgraphgenerators SHARED grid.cpp voronoi.cpp generator.cpp)
depend_on_eigen3(fawkesnavgraphgenerators)
depend_on_navgraph(fawkesnavgraphgenerators)
depend_on_cgal_libs(fawkesnavgraphgenerators)
target_link_libraries(fawkesnavgraphgenerators fawkescore fawkesaspects
                      fawkesutils fawkesnavgraph)
