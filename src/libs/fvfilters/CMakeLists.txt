set(fvflters_SOURCES)
list(
  APPEND
  fvfilters_SOURCES
  difference.cpp
  threshold.cpp
  hipass.cpp
  rectify.cpp
  compare.cpp
  roidraw.cpp
  # qa/qa_gauss.cpp qa/qa_sobel.cpp qa/qa_erode.cpp qa/qa_sharpen.cpp
  sharpen.cpp
  sobel.cpp
  filter.cpp
  sum.cpp
  invert.cpp
  median.cpp
  hv_search.cpp
  segment_color.cpp
  min.cpp
  laplace.cpp
  hor_search.cpp
  shape_remover.cpp
  segment.cpp
  gauss.cpp
  colorthreshold.cpp
  segment_scanline.cpp
  unwarp.cpp
  max.cpp
  or.cpp)

if(OPENCV_FOUND)
  list(
    APPEND
    fvfilters_SOURCES
    morphology/closing.cpp
    morphology/morphologicalfilter.cpp
    morphology/erosion.cpp
    morphology/tophat_closing.cpp
    morphology/geodesic_erosion.cpp
    morphology/segenerator.cpp
    morphology/dilation.cpp
    morphology/opening.cpp
    morphology/geodesic_dilation.cpp)
else()
  message("Omitting filters requiring OpenCV (opencv[-devel] not installed)")
endif()

add_library(fvfilters SHARED ${fvfilters_SOURCES})
depend_on_fvconf(fvfilters)
target_link_libraries(fvfilters fawkescore fawkesutils fvutils)
# include nothing.h
