set(fvclassifiers_SOURCES)
list(
  APPEND
  fvclassifiers_SOURCES
  qualifiers.cpp
  # qa/qa_siftppclassifier.cpp
  # qa/qa_facesclassifier.cpp qa/qa_surfclassifier.cpp qa/qa_siftclassifier.cpp
  square_shrinker.cpp
  simple.cpp
  classifier.cpp
  multi_color.cpp
  # sift.cpp # outdated
  # surf.cpp # outdated siftpp.cpp # outdated
  gradient.cpp
  shrinker.cpp
  border_shrinker.cpp)

if(OPENCV_FOUND)
  list(APPEND fvclassifiers_SOURCES faces.cpp)
else()
  message(WARNING "Omitting face detector (opencv[-devel] not installed)")
endif()

add_library(fvclassifiers SHARED ${fvclassifiers_SOURCES})
depend_on_fvconf(fvclassifiers)
target_link_libraries(fvclassifiers fawkescore fawkesutils)
if(OPENCV_FOUND)
  target_link_libraries(fvclassifiers -lopencv_objdetect)
endif()
