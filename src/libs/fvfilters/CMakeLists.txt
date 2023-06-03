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
  fvfilters SHARED
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
depend_on_fvconf(fvfilters)

if(OPENCV_FOUND)
  target_sources(
    fvfilters
    PRIVATE morphology/closing.cpp
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

target_link_libraries(fvfilters fawkescore fawkesutils fvutils)
# include nothing.h
