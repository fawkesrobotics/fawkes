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

set(PLUGIN_flite
    ON
    CACHE BOOL "Build flite plugin")

if(PLUGIN_flite)
  find_path(FLITE_ROOT flite/flite.h PATH_SUFFIXES include)

  add_library(flite MODULE flite_plugin.cpp synth_thread.cpp)

  depend_on_pkgconfig_libs(flite alsa)

  target_link_libraries(
    flite
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    SpeechSynthInterface
    FLite)

  if(NOT FLITE_ROOT)
    disable_taget(flite)
    build_skipped_message("flite" "flite[-devel] dependency")
  endif()
else()
  plugin_disabled_message(flite)
endif()
