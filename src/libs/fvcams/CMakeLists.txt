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
  fvcams SHARED
  # kinect.cpp # was never built?
  factory.cpp
  # qa/qa_fwcam.cpp
  control/factory.cpp
  control/zoom.cpp
  # control/sony_evid100p.cpp # outdated
  control/pantilt.cpp
  control/focus.cpp
  # control/visca.cpp # outdated
  control/color.cpp
  control/image.cpp
  control/effect.cpp
  control/control.cpp
  control/source.cpp
  control/dummy.cpp
  buffer.cpp
  cam_exceptions.cpp
  camera.cpp
  # leutron.cpp # outdated nao.cpp # outdated v4l1.cpp # outdated
  net.cpp
  fileloader.cpp
  shmem.cpp)

depend_on_fvconf(fvcams)
if(FIREWIRE_CAM_FOUND)
  target_sources(fvcams PRIVATE firewire.cpp bumblebee2.cpp pike.cpp)
endif()

if(V4L2_CAM_FOUND)
  target_sources(fvcams PRIVATE v4l2.cpp v4l.cpp)
endif()
target_link_libraries(fvcams fawkescore fawkesutils fvutils fawkeslogging)
