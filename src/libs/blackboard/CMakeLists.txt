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

include_directories(libs)
add_library(
  fawkesblackboard SHARED
  interface_observer.cpp
  net/interface_observer.cpp
  net/interface_listener.cpp
  net/ilist_content.cpp
  net/interface_proxy.cpp
  net/handler.cpp
  shmem/lister.cpp
  shmem/header.cpp
  interface_list_maintainer.cpp
  internal/memory_manager.cpp
  internal/interface_manager.cpp
  internal/notifier.cpp
  internal/instance_factory.cpp
  internal/message_manager.cpp
  # qa/qa_bb_buffers.cpp
  # qa/qa_bb_notify.cpp
  # qa/qa_bb_listall.cpp
  # qa/qa_bb_messaging.cpp
  # qa/qa_bb_remote.cpp
  # qa/qa_bb_openall.cpp
  # qa/qa_bb_objpos.cpp
  # qa/qa_bb_memmgr.cpp
  # qa/qa_bb_interface.cpp
  interface_listener.cpp
  utils/on_message_waker.cpp
  utils/on_update_waker.cpp
  remote.cpp
  local.cpp
  blackboard.cpp
  ownership.cpp)

target_link_libraries(fawkesblackboard fawkescore fawkesutils fawkesnetcomm
                      fawkeslogging fawkesinterface)

set(LUA_SOURCES blackboard.tolua)
set(LUA_LINK_LIBS fawkescore fawkesutils fawkesblackboard)
generate_lua_lib(fawkesblackboard "${LUA_SOURCES}" "${LUA_LINK_LIBS}")
