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

set(PLUGIN_static-transforms
    ON
    CACHE BOOL "Build static-transforms plugin")

if(PLUGIN_static-transforms)

  add_library(static-transforms MODULE static_transforms_plugin.cpp
                                       static_transforms_thread.cpp)
  depend_on_tf(static-transforms)

  target_link_libraries(
    static-transforms
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    fawkestf
    fawkesconfig)

else()
  plugin_disabled_message("static-transforms")
endif()
