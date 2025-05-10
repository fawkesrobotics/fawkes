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

find_package(Protobuf)

# Function: depend_on_protobuf
# Usage: depend_on_protobuf(TARGET_NAME)
#
# Adds the dependency for the Protobuf Libaries to the target.
function(depend_on_protobuf target)
  if(Protobuf_FOUND)
    target_link_libraries(${target} ${Protobuf_LIBRARIES})
    target_include_directories(${target} PUBLIC ${Protobuf_INCLUDE_DIRS})
  else()
    disable_target({target})
    build_skipped_message(${target} "Protobuf")
  endif()
endfunction()

# Function: depend_on_protobuff_comm
# Usage: depend_on_protobuff_comm
#
# Adds the dependency for the protofbuff_comm package to the target.
function(depend_on_protobuf_comm target)
  depend_on_find_package_libs(${target} ProtobufComm)
  if(ProtobufComm_FOUND)
    target_link_libraries(${target}  ProtobufComm::protobuf_comm)
  endif()
endfunction()
