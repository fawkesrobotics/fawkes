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

add_library(fawkesprotoboard SHARED protobuf_thread.cpp blackboard_manager.cpp
                                    protobuf_to_bb.cpp)
depend_on_protobuf(fawkesprotoboard)
depend_on_protobuf_comm(fawkesprotoboard)
target_link_libraries(fawkesprotoboard fawkescore fawkesaspects
                      fawkesblackboard ProtobufPeerInterface)
