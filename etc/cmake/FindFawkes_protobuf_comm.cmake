# - Fawkes: check for protobuf_comm
#
# Once done this will define
#
#  Fawkes_protobuf_comm_FOUND - component was found
#  Fawkes_protobuf_comm_CFLAGS - Flags to add to get protobuf_comm support
#  Fawkes_protobuf_comm_LIBRARIES - Libs required for protobuf_comm support
#
# Copyright (c) 2015  Tim Niemueller [www.niemueller.de]
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. The name of the author may not be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

if (NOT Fawkes_FOUND)
  message(ERROR "Cannot check for component if Fawkes not found")
  return()
endif()

find_package(Fawkes_CPP11 REQUIRED)
find_package(Protobuf REQUIRED QUIET)
find_library(Fawkes_protobuf_comm_LIBRARIES fawkes_protobuf_comm HINTS "${Fawkes_BASEDIR}/lib")

if (NOT PROTOBUF_PROTOC_LIBRARIES)
  message(FATAL_ERROR "Fawkes protobuf_comm requires the protoc libraries to be available")
endif()


list(FIND PROTOBUF_LIBRARIES "optimized" _PROTOBUF_LIBRARY_OPTIMIZED_IDX)
if (_PROTOBUF_LIBRARY_OPTIMIZED_IDX GREATER -1)
  # debug and release libraries, only choose release library
  # (if we do not do this cmake screws up when we add lists later)
  math(EXPR _PROTOBUF_LIBRARY_OPTIMIZED_IDX "${_PROTOBUF_LIBRARY_OPTIMIZED_IDX} + 1")
  list(GET PROTOBUF_LIBRARIES ${_PROTOBUF_LIBRARY_OPTIMIZED_IDX} PROTOBUF_LIBRARIES)
endif()

list(FIND PROTOBUF_PROTOC_LIBRARIES "optimized" _PROTOBUF_PROTOC_LIBRARY_OPTIMIZED_IDX)
if (_PROTOBUF_PROTOC_LIBRARY_OPTIMIZED_IDX GREATER -1)
  # debug and release libraries, only choose release library
  # (if we do not do this cmake screws up when we add lists later)
  math(EXPR _PROTOBUF_PROTOC_LIBRARY_OPTIMIZED_IDX "${_PROTOBUF_PROTOC_LIBRARY_OPTIMIZED_IDX} + 1")
  list(GET PROTOBUF_PROTOC_LIBRARIES ${_PROTOBUF_PROTOC_LIBRARY_OPTIMIZED_IDX} PROTOBUF_PROTOC_LIBRARIES)
endif()

if (Fawkes_protobuf_comm_LIBRARIES)
  set(Fawkes_protobuf_comm_FOUND TRUE)
  set(Fawkes_protobuf_comm_CFLAGS "${Fawkes_CPP11_CFLAGS} -I${PROTOBUF_INCLUDE_DIR}")
  set(Fawkes_protobuf_comm_LIBRARY_DIRS "${Fawkes_LIBDIR}/protobuf")
  set(Fawkes_protobuf_comm_LIBRARIES fawkes_protobuf_comm ${PROTOBUF_LIBRARIES} ${PROTOBUF_PROTOC_LIBRARIES})
endif()
