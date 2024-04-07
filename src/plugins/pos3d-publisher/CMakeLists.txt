# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2024 by Tarik Viehmann
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

set(PLUGIN_pos3d-publisher
    ON
    CACHE BOOL "Build pos3d-publisher plugin")

# find_package(Boost COMPONENTS system filesystem)

# add_subdirectory(interfaces)
if(PLUGIN_pos3d-publisher)
    add_library(pos3d-publisher MODULE pos3d_publisher_plugin.cpp
        pos3d_publisher_thread.cpp)

    depend_on_tf(pos3d-publisher)
    target_link_libraries(
        pos3d-publisher
        fawkescore
        fawkesutils
        fawkesaspects
        fawkesbaseapp
        fawkesblackboard
        fawkesinterface
        fawkestf)

    target_compile_features(pos3d-publisher PUBLIC cxx_std_17)

else()
    plugin_disabled_message("pos3d-publisher")
endif()
