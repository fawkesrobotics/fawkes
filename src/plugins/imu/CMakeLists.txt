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

set(PLUGIN_imu
    ON
    CACHE BOOL "Build imu plugin")

if(PLUGIN_imu)
  add_library(imu MODULE imu_plugin.cpp acquisition_thread.cpp
                         sensor_thread.cpp)

  optional_depend_on_boost_libs(imu "thread;system" boost_deps_found)
  if(boost_deps_found)

    target_sources(imu PRIVATE imu_cruizcore_xg1010.cpp)
    target_link_libraries(imu ${Boost_LIBRARIES})
    target_include_directories(imu PRIVATE ${Boost_INCLUDE_DIRS})
    depend_on_tf(imu)
    target_compile_definitions(imu PUBLIC HAVE_CRUIZCORE)
  endif()

  target_link_libraries(
    imu
    m
    pthread
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    IMUInterface)

else()
  plugin_disabled_message("imu")
endif()
