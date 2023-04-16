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
  fvwidgets SHARED image_widget.cpp fuse_image_list_widget.cpp
  # qa/qa_image_display.cpp
)
optional_depend_on_pkgconfig_libs(fvwidgets sdl sdl_dep_found)
if(sdl_dep_found)
  target_sources(fvwidgets PRIVATE sdl_keeper.cpp image_display.cpp)
else()
  build_skipped_message("fvwidgets SDL Widgets" "sdl12-compat[-devel]")
endif()

depend_on_fvconf(fvwidgets)
depend_on_gui(fvwidgets)
target_link_libraries(fvwidgets fawkescore fawkesutils fvutils)
