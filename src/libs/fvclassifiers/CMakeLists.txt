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
