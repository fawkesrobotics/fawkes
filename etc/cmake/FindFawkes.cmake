# - Find the Fawkes Robotics Software Framework (http://www.fawkesrobotics.org)
#
# Once done this will define
#
#  FAWKES_FOUND - Fawkes was found
#  FAWKES_INCLUDE_DIRS - Fawkes include directories
#  FAWKES_LIBRARY_DIRS - Link directories for Fawkes libraries


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

#set(FAWKES_DIR "$ENV{HOME}/robotics/tino-fawkes")

# Need that for many components
include(FindPkgConfig)

message(STATUS "Checking for Fawkes")

# These are components for which we do not need extra modules which
# only require basic cflags and ldflags
set(FAWKES_BASE_COMPONENTS aspect baseapp blackboard core interface utils logging netcomm)

if (NOT FAWKES_DIR)
  if (NOT "$ENV{FAWKES_DIR}" STREQUAL "")
    #message(STATUS "  checking env var dir $ENV{FAWKES_DIR}")
    # if the FAWKES_DIR environment variable is set first look there
    find_path(FAWKES_DIR version.h
      PATHS ENV FAWKES_DIR
      PATH_SUFFIXES fawkes/src/libs/core src/libs/core
      NO_DEFAULT_PATH)
  endif()

  if (NOT FAWKES_DIR)
    # try to find Fawkes in a few common paths
    find_path(FAWKES_DIR
      version.h
      PATHS $ENV{HOME}/fawkes $ENV{HOME}/fawkes-robotino $ENV{HOME}/fawkes-athome
            $ENV{HOME}/robotics/fawkes $ENV{HOME}/robotics/fawkes-robotino
            $ENV{HOME}/robotics/fawkes-athome
      PATH_SUFFIXES fawkes/src/libs/core src/libs/core
    )
  endif()
endif()

if (FAWKES_DIR)
  if (FAWKES_DIR MATCHES "src/libs/core$")
    get_filename_component(FAWKES_DIR "${FAWKES_DIR}/../../.." ABSOLUTE)
  endif()
  if (EXISTS "${FAWKES_DIR}/fawkes/etc/buildsys/config.mk")
    get_filename_component(Fawkes_BASEDIR "${FAWKES_DIR}" ABSOLUTE)
    set(FAWKES_DIR "${FAWKES_DIR}/fawkes")
    set(Fawkes_IS_SUBMODULE TRUE)
  elseif (EXISTS "${FAWKES_DIR}/../etc/buildsys/config.mk")
    get_filename_component(Fawkes_BASEDIR "${FAWKES_DIR}/.." ABSOLUTE)
    set(Fawkes_IS_SUBMODULE TRUE)
  else()
    get_filename_component(Fawkes_BASEDIR "${FAWKES_DIR}" ABSOLUTE)
  endif()

  set(Fawkes_COREDIR ${FAWKES_DIR})
  
  message(STATUS "  base dir: ${Fawkes_BASEDIR}")
  if (Fawkes_IS_SUBMODULE)
    message(STATUS "  sub-module, core dir: ${Fawkes_COREDIR}")
  endif()
endif()

if (Fawkes_BASEDIR)
  # parse version
  file(STRINGS "${Fawkes_COREDIR}/src/libs/core/version.h" FAWKES_VERSION_FILE)
  foreach (LINE ${FAWKES_VERSION_FILE})
    string(REGEX MATCH "^#define FAWKES_VERSION_MAJOR  ([0-9]+)$" MATCH "${LINE}")
    if (MATCH)
      set(Fawkes_VERSION_MAJOR ${CMAKE_MATCH_1})
    endif()
    string(REGEX MATCH "^#define FAWKES_VERSION_MINOR  ([0-9]+)$" MATCH "${LINE}")
    if (MATCH)
      set(Fawkes_VERSION_MINOR ${CMAKE_MATCH_1})
    endif()
    string(REGEX MATCH "^#define FAWKES_VERSION_MICRO  ([0-9]+)$" MATCH "${LINE}")
    if (MATCH)
      set(Fawkes_VERSION_MICRO ${CMAKE_MATCH_1})
    endif()
  endforeach()
  message(STATUS "  version: ${Fawkes_VERSION_MAJOR}.${Fawkes_VERSION_MINOR}.${Fawkes_VERSION_MICRO}")

  set(Fawkes_FOUND TRUE)
  set(Fawkes_BINDIR ${Fawkes_BASEDIR}/bin)
  set(Fawkes_LIBDIR ${Fawkes_BASEDIR}/lib)
  set(Fawkes_RESDIR ${Fawkes_BASEDIR}/res)
  set(Fawkes_CONFDIR ${Fawkes_BASEDIR}/cfg)
  set(Fawkes_PLUGINDIR ${Fawkes_BASEDIR}/plugins)
  set(Fawkes_IFACEDIR ${Fawkes_BASEDIR}/lib/interfaces)
  set(Fawkes_USERDIR ".fawkes")
  set(Fawkes_TMPDIR "/tmp")
  set(Fawkes_LOGDIR ${Fawkes_BASEDIR}/log)
  set(Fawkes_INCLUDE_DIRS
    ${Fawkes_BASEDIR}/src ${Fawkes_COREDIR}/src
    ${Fawkes_BASEDIR}/src/libs ${Fawkes_COREDIR}/src/libs)
  set(Fawkes_LIBRARY_DIRS ${Fawkes_LIBDIR} ${Fawkes_IFACEDIR})
  set(Fawkes_CFLAGS "-fPIC -pthread")
  set(Fawkes_CFLAGS "${Fawkes_CFLAGS} -DBASEDIR=\\\"${Fawkes_BASEDIR}\\\" -DFAWKES_BASEDIR=\\\"${Fawkes_COREDIR}\\\"")
  set(Fawkes_CFLAGS "${Fawkes_CFLAGS} -DBINDIR=\\\"${Fawkes_BINDIR}\\\" -DLIBDIR=\\\"${Fawkes_LIBDIR}\\\"")
  set(Fawkes_CFLAGS "${Fawkes_CFLAGS} -DPLUGINDIR=\\\"${Fawkes_PLUGINDIR}\\\" -DIFACEDIR=\\\"${Fawkes_IFACEDIR}\\\"")
  set(Fawkes_CFLAGS "${Fawkes_CFLAGS} -DCONFDIR=\\\"${Fawkes_CONFDIR}\\\" -DUSERDIR=\\\"${Fawkes_USERDIR}\\\"")
  set(Fawkes_CFLAGS "${Fawkes_CFLAGS} -DLOGDIR=\\\"${Fawkes_LOGDIR}\\\" -DRESDIR=\\\"${Fawkes_RESDIR}\\\"")
  set(Fawkes_CFLAGS "${Fawkes_CFLAGS} -DTMPDIR=\\\"${Fawkes_TMPDIR}\\\"")

  foreach (COMPONENT ${Fawkes_FIND_COMPONENTS})
    list(FIND FAWKES_BASE_COMPONENTS ${COMPONENT} _BFOUND)
    string(REGEX MATCH "^[a-zA-Z0-9_-]+Interface$" IS_INTERFACE_MATCH ${COMPONENT})
    if (_BFOUND GREATER -1)
      # It's a base component, just check for library
      find_library(Fawkes_${COMPONENT} fawkes${COMPONENT} HINTS ${Fawkes_LIBDIR})
      if (Fawkes_${COMPONENT})
	message(STATUS "  found Fawkes component ${COMPONENT}")
	list(APPEND Fawkes_LIBRARIES fawkes${COMPONENT})
      else()
	message(FATAL_ERROR "  Fawkes component ${COMPONENT} **NOT** found")
      endif()
    elseif (IS_INTERFACE_MATCH)
      find_library(Fawkes_${COMPONENT} ${COMPONENT} HINTS ${Fawkes_IFACEDIR})
      if (Fawkes_${COMPONENT})
	message(STATUS "  found Fawkes interface ${COMPONENT}")
	list(APPEND Fawkes_LIBRARIES ${COMPONENT})
      else()
	message(FATAL_ERROR "  Fawkes interface ${COMPONENT} **NOT** found")
      endif()
    else()
      # It's a component that requires its own module
      find_package(Fawkes_${COMPONENT} REQUIRED)
      if (Fawkes_${COMPONENT}_FOUND)
	message(STATUS "  found Fawkes component ${COMPONENT}")
	list(APPEND Fawkes_LIBRARIES ${Fawkes_${COMPONENT}_LIBRARIES})
	list(APPEND Fawkes_INCLUDE_DIRS ${Fawkes_${COMPONENT}_INCLUDE_DIRS})
	list(APPEND Fawkes_LIBRARY_DIRS ${Fawkes_${COMPONENT}_LIBRARY_DIRS})
        set(Fawkes_CFLAGS "${Fawkes_CFLAGS} ${Fawkes_${COMPONENT}_CFLAGS}")
	set(Fawkes_LFLAGS "${Fawkes_LFLAGS} ${Fawkes_${COMPONENT}_LFLAGS}")
      else()
	message(FATAL_ERROR "  Fawkes component ${COMPONENT} **NOT** found")
      endif()
    endif()
  endforeach()

  foreach (LD ${Fawkes_LIBRARY_DIRS})
    set(Fawkes_LFLAGS "${Fawkes_LFLAGS} -L${LD}")
  endforeach()

  string(STRIP "${Fawkes_CFLAGS}" Fawkes_CFLAGS)
  string(STRIP "${Fawkes_LFLAGS}" Fawkes_LFLAGS)
endif()
