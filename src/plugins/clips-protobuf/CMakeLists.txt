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

set(PLUGIN_clips-protobuf
    ON
    CACHE BOOL "Build clips-protobuf plugin")

if(PLUGIN_clips-protobuf)
  add_library(clips-protobuf MODULE clips-protobuf-plugin.cpp
                                    clips-protobuf-thread.cpp)
  target_compile_features(clips-protobuf PUBLIC cxx_std_11)
  depend_on_clipsmm(clips-protobuf)
  depend_on_protobuf(clips-protobuf)
  depend_on_boost_libs(clips-protobuf system)
  target_link_libraries(clips-protobuf fawkescore fawkesutils fawkesaspects
                        fawkesclipsaspect fawkes_protobuf_clips)

else()
  plugin_disabled_message("clips-protobuf")
endif()
