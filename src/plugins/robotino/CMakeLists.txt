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

set(PLUGIN_robotino
    ON
    CACHE BOOL ON)

if(PLUGIN_robotino)
  find_package(Boost COMPONENTS thread filesystem)
  add_subdirectory(interfaces)

  add_library(
    robotino MODULE robotino_plugin.cpp com_thread.cpp sensor_thread.cpp
                    act_thread.cpp direct_com_thread.cpp direct_com_message.cpp)
  # openrobotino_com_thread.cpp # if openrobotino is available
  target_compile_definitions(robotino PUBLIC HAVE_ROBOTINO_DIRECT)

  depend_on_boost_libs(robotino "thread;filesystem")
  depend_on_pkgconfig_libs(robotino libudev)

  target_compile_features(robotino PUBLIC cxx_std_11)
  target_link_libraries(robotino ${LIBUDEV_LDFLAGS})
  target_compile_options(robotino PUBLIC ${LIBUDEV_CFLAGS})
  target_compile_definitions(robotino PUBLIC HAVE_LIBUDEV)
  target_link_libraries(robotino ${Boost_LIBRARIES})
  target_include_directories(robotino PUBLIC ${Boost_INCLUDE_DIRS})

  optional_depend_on_pkgconfig_libs(robotino "${tf_deps}" tf_deps_found)
  if(tf_deps_found)
    depend_on_tf(robotino)
    set_source_files_properties(
      sensor_thread.cpp PROPERTIES COMPILE_OPTIONS
                                   "-Wno-reorder -Wno-unused-function")
  endif()

  target_link_libraries(
    robotino
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    MotorInterface
    GripperInterface
    BatteryInterface
    RobotinoSensorInterface
    IMUInterface)
else()
  plugin_disabled_message(robotino)
endif()
