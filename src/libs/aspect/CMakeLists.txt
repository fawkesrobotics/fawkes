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
  fawkesaspects SHARED
  syncpoint.cpp
  logging.cpp
  plugin_director.cpp
  vision.cpp
  syncpoint_manager.cpp
  fawkes_network.cpp
  aspect_provider.cpp
  thread_producer.cpp
  time_source.cpp
  blocked_timing/executor.cpp
  configurable.cpp
  manager.cpp
  inifins/syncpoint.cpp
  inifins/logging.cpp
  inifins/plugin_director.cpp
  inifins/vision.cpp
  inifins/syncpoint_manager.cpp
  inifins/fawkes_network.cpp
  inifins/aspect_provider.cpp
  inifins/inifin.cpp
  inifins/thread_producer.cpp
  inifins/time_source.cpp
  inifins/configurable.cpp
  inifins/blocked_timing.cpp
  inifins/blackboard.cpp
  inifins/vision_master.cpp
  inifins/clock.cpp
  inifins/mainloop.cpp
  inifins/network.cpp
  inifins/logger.cpp
  blocked_timing.cpp
  aspect.cpp
  mainloop/employer.cpp
  blackboard.cpp
  vision_master.cpp
  clock.cpp
  mainloop.cpp
  network.cpp
  logger.cpp)

if(PCL_FOUND)
  target_sources(fawkesaspects PRIVATE pointcloud.cpp inifins/pointcloud.cpp)
  depend_on_pcl(fawkesaspects)
  target_link_libraries(fawkesaspects fawkespcl_utils)
endif()

optional_depend_on_pkgconfig_libs(fawkesaspects "${tf_deps}" tf_deps_found)
if(tf_deps_found)
  target_sources(fawkesaspects PRIVATE tf.cpp inifins/tf.cpp)
  depend_on_tf(fawkesaspects)
  target_link_libraries(fawkesaspects fawkestf)
endif()

optional_depend_on_pkgconfig_libs(fawkesaspects "${webview_deps}"
                                  webview_deps_found)
if(webview_deps_found)
  target_sources(fawkesaspects PRIVATE webview.cpp inifins/webview.cpp)
  depend_on_webview(fawkesaspects)
  target_link_libraries(fawkesaspects fawkeswebview)
endif()

target_link_libraries(
  fawkesaspects
  fawkescore
  fawkesutils
  fawkesblackboard
  fawkesnetcomm
  fawkesconfig
  fawkesplugin
  fawkeslogging
  fawkessyncpoint)
