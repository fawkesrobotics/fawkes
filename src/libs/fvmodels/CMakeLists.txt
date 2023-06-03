# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann and Daniel Swoboda
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************

add_library(
  fvmodels SHARED
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

depend_on_fvconf(fvmodels)

if(OPENCV_FOUND)
  target_sources(fvmodels PRIVATE mirror/mirror_calib.cpp)
endif()

optional_depend_on_pkgconfig_libs(fvmodels "${TF_DEPS}" TF_DEPS_FOUND)
if(TF_DEPS_FOUND)
  target_sources(fvmodels PRIVATE relative_position/position_to_pixel.cpp)
  depend_on_tf(fvmodels)
  target_link_libraries(fvmodels fawkestf)
  build_depends_on(fvmodels fawkestf)
endif()

target_link_libraries(fvmodels fawkescore fawkesutils fvfilters)
