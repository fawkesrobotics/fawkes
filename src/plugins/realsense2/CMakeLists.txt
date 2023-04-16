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

set(PLUGIN_realsense2 ON CACHCE BOOL ON)
pkg_check_modules(realsense2 realsense2>=2.16.0)
remember_dependency(realsense2)

if(PLUGIN_realsense2)

  add_library(realsense2 MODULE realsense2_plugin.cpp realsense2_thread.cpp)

  depend_on_pcl(realsense2)
  depend_on_tf(realsense2)

  depend_on_pkgconfig_libs(realsense2 realsense2)

  target_compile_options(realsense2 PUBLIC ${REALSENSE2_CFLAGS})
  target_link_libraries(
    realsense2
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    fvutils
    fawkespcl_utils
    SwitchInterface
    ${REALSENSE2_LDFLAGS})
else()
  plugin_disabled_message("realsense2")
endif()
