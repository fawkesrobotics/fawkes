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
  fawkesgui_utils SHARED
  service_model.cpp
  connection_dispatcher.cpp
  service_chooser_dialog.cpp
  interface_chooser_dialog.cpp
  twolines_cellrenderer.cpp
  logview.cpp
  interface_dispatcher.cpp
  avahi_dispatcher.cpp
  multi_interface_chooser_dialog.cpp
  service_selector_cbe.cpp
  plugin_tree_view.cpp
  robot/drawer.cpp
  robot/allemaniacs_athome.cpp)

depend_on_gui(fawkesgui_utils)

target_link_libraries(
  fawkesgui_utils
  stdc++
  m
  fawkescore
  fawkesnetcomm
  fawkesblackboard
  fawkesinterface
  fawkesplugin
  fawkeslogging
  fawkesnetwork_logger
  fawkesutils)
