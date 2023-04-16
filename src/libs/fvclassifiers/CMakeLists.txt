add_library(
  fvclassifiers SHARED
  qualifiers.cpp
  # qa/qa_siftppclassifier.cpp qa/qa_facesclassifier.cpp
  # qa/qa_surfclassifier.cpp qa/qa_siftclassifier.cpp
  square_shrinker.cpp
  simple.cpp
  classifier.cpp
  multi_color.cpp
  # sift.cpp # outdated surf.cpp # outdated siftpp.cpp # outdated
  gradient.cpp
  shrinker.cpp
  border_shrinker.cpp)

optional_depend_on_pkgconfig_libs(fvclassifiers opencv opencv_dep_found)
if(opencv_dep_found)
  target_sources(fvclassifiers PRIVATE faces.cpp)
  target_link_libraries(fvclassifiers -lopencv_objdetect)
endif()
depend_on_fvconf(fvclassifiers)
target_link_libraries(fvclassifiers fawkescore fawkesutils)
