# - Fawkes: check for navgraph
#
# Once done this will define
#
#  Fawkes_navgraph_FOUND - component was found
#  Fawkes_navgraph_CFLAGS - Flags to add to get navgraph support
#  Fawkes_navgraph_LIBRARIES - Libs required for navgraph support
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
pkg_check_modules(YAML_CPP REQUIRED QUIET yaml-cpp)
find_library(Fawkes_navgraph_LIBRARIES fawkesnavgraph HINTS "${Fawkes_BASEDIR}/lib")

if (Fawkes_navgraph_LIBRARIES)
  set(Fawkes_navgraph_FOUND TRUE)
  set(Fawkes_navgraph_CFLAGS "${Fawkes_CPP11_CFLAGS} ${YAML_CPP_CFLAGS}")
  set(Fawkes_navgraph_LIBRARIES fawkesnavgraph m)
endif()
