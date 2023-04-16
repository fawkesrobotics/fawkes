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

set(PLUGIN_laser
    ON
    CACHE BOOL ON)
if(PLUGIN_laser)
  add_library(laser MODULE laser_plugin.cpp acquisition_thread.cpp
                           sensor_thread.cpp)
  depend_on_laser(laser)
  if(URG_ROOT)
    target_sources(laser PRIVATE urg_aqt.cpp)
  else()
    build_skipped_message("laser plugin URG support" "urg[-devel] dependency")
  endif()
  if(URG_GBX_FOUND)
    target_sources(laser PRIVATE urg_gbx_aqt.cpp)
  else()
    build_skipped_message("laser plugin Hokuyo via Gearbox support"
                          "flexiport[-devel] or hokuyoaist[-devel] dependency")
  endif()

  if(LIBUSB_FOUND OR SICK_TIM55X_BOOST_DEPS_FOUND)
    target_sources(laser PRIVATE sick_tim55x_common_aqt.cpp)
    if(LIBUSB_FOUND)
      target_sources(laser PRIVATE sick_tim55x_usb_aqt.cpp)
    else()
      build_skipped_message("laser plugin Sick TiM55x-USB support"
                            "libusb1[-devel] dependency")
    endif()

    if(SICK_TIM55X_BOOST_DEPS_FOUND)
      target_sources(laser PRIVATE sick_tim55x_ethernet_aqt.cpp)
    else()
      build_skipped_message("laser plugin TiM55x-Ethernet support "
                            "Boost system or thread dependency")
    endif()
  endif()
  target_link_libraries(
    laser
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    Laser360Interface
    Laser720Interface
    Laser1080Interface)
else()
  plugin_disabled_message(laser)
endif()
