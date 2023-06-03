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

set(PLUGIN_fvfountain
    ON
    CACHE BOOL "Build fvfountain plugin")

if(PLUGIN_fvfountain)
  add_library(fvfountain MODULE fountain_plugin.cpp fountain_thread.cpp)
  # mini_image_producer.cpp is outdated code, excluded here

  depend_on_fvconf(fvfountain)

  target_link_libraries(
    fvfountain
    fawkescore
    fawkesutils
    fvutils
    fawkesnetcomm
    fawkesaspects
    fawkeslogging)

else()
  plugin_disabled_message(fvfountain)
endif()
