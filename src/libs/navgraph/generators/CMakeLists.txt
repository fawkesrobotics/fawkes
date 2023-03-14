set(NAVGRAPH_FOUND 0)
set(CGAL_LIBS_FOUND 0)
check_navgraph_deps(NAVGRAPH_FOUND)
check_cgal_libs_deps(CGAL_LIBS_FOUND)

if(NAVGRAPH_FOUND
   AND CGAL_LIBS_FOUND
   AND EIGEN3_FOUND)
  add_library(fawkesnavgraphgenerators SHARED grid.cpp voronoi.cpp
                                              generator.cpp)
  depend_on_eigen3(fawkesnavgraphgenerators)
  depend_on_navgraph(fawkesnavgraphgenerators)
  depend_on_cgal_libs(fawkesnavgraphgenerators)
  target_link_libraries(fawkesnavgraphgenerators fawkescore fawkesaspects
                        fawkesutils fawkesnavgraph)
else()
  message(WARNING "Skipped due to missing dependencies")
endif()
