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

add_library(fawkesplugin SHARED net/list_message.cpp net/handler.cpp
                                listener.cpp manager.cpp loader.cpp)

target_compile_options(fawkesconfig PUBLIC -DHAVE_LIBELF)

if(EXISTS "/usr/include/sys/inotify.h")
  target_compile_options(fawkesconfig PUBLIC -DHAVE_INOTIFY)
endif()

target_link_libraries(
  fawkesplugin
  stdc++
  elf
  fawkescore
  fawkesutils
  fawkesconfig
  fawkesnetcomm
  fawkeslogging
  dl)
