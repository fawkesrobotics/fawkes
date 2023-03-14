set(fvmodels_SOURCES)
list(
  APPEND
  fvmodels_SOURCES
  relative_position/box_relative.cpp
  relative_position/ball_trigo.cpp
  relative_position/relativepositionmodel.cpp
  relative_position/omni_relative.cpp
  mirror/mirrormodel.cpp
  mirror/bulb.cpp
  velocity/globvelo.cpp
  velocity/velocitymodel.cpp
  velocity/globfromrel.cpp
  velocity/relvelo.cpp
  scanlines/star.cpp
  scanlines/cornerhorizon.cpp
  scanlines/beams.cpp
  scanlines/grid.cpp
  scanlines/line_grid.cpp
  scanlines/radial.cpp
  color/thresholds_luminance.cpp
  color/thresholds_black.cpp
  color/similarity.cpp
  color/thresholds.cpp
  color/colormodel.cpp
  color/lookuptable.cpp
  global_position/omni_global.cpp
  global_position/globalpositionmodel.cpp
  global_position/globfromrel.cpp
  shape/line.cpp
  shape/rcd_circle.cpp
  shape/rht_circle.cpp
  shape/accumulators/fc_accum.cpp
  shape/accumulators/ht_accum.cpp
  shape/accumulators/fit_accum.cpp
  shape/shapemodel.cpp
  shape/ht_lines.cpp
  shape/rht_lines.cpp
  shape/circle.cpp)

if(OPENCV_FOUND)
  list(APPEND fvmodels_SOURCES mirror/mirror_calib.cpp)
endif()

if(TF_FOUND)
  list(APPEND fvmodels_SOURCES relative_position/position_to_pixel.cpp)
endif()

add_library(fvmodels SHARED ${fvmodels_SOURCES})
if(TF_FOUND)
  depend_on_tf(fvmodels)
  target_link_libraries(fvmodels fawkestf)
endif()
depend_on_fvconf(fvmodels)
target_link_libraries(fvmodels fawkescore fawkesutils fvfilters)
