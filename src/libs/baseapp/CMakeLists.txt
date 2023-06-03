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
  fawkesbaseapp SHARED thread_manager.cpp timing_thread.cpp main_thread.cpp
                       daemonize.cpp init_options.cpp run.cpp)

target_link_libraries(
  fawkesbaseapp
  stdc++
  pthread
  fawkescore
  fawkesutils
  fawkesconfig
  fawkesplugin
  fawkesaspects
  fawkeslogging
  fawkessyncpoint
  fawkesnetcomm
  fawkesnetwork_logger)

depend_on_boost_libs(fawkesbaseapp system)

target_compile_options(
  fawkesbaseapp
  PUBLIC -DHAVE_NETWORK_MANAGER
         -DHAVE_NETWORK_LOGGER
         -DHAVE_CONFIG_NETWORK_HANDLER
         -DHAVE_PLUGIN_NETWORK_HANDLER
         -DHAVE_LOGGING_FD_REDIRECT
         -DHAVE_BLACKBOARD
         -DHAVE_BOOST
         -DHAVE_CPP11)
if(TF_FOUND)
  depend_on_tf(fawkesbaseapp)
endif()
