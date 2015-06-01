# - Fawkes: check for ROS integration
#
# Once done this will define
#
#  Fawkes_ros_FOUND - component was found
#  Fawkes_ros_CFLAGS - Flags to add to get ROS support
#  Fawkes_ros_LIBRARIES - Libs required for ROS support
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
  message(FATAL_ERROR "Cannot check for component if Fawkes not found")
  return()
endif()

find_program(ROSPACK rospack PATHS ENV PATH)
if (NOT ROSPACK)
  message(WARNING "Cannot find rospack")
  set(Fawkes_ros_FOUND FALSE)
  return()
endif()

# Sets ${PKGNAME}_PATH
function (ros_pkg_path PKGNAME)
  execute_process(COMMAND ${ROSPACK} find ${PKGNAME} ERROR_QUIET OUTPUT_VARIABLE PKGPATH)
  if ("${PKGPATH}" STREQUAL "")
    message(FATAL_ERROR "ROS package ${PKGNAME} could not be found")
  endif()
  set(${PKGNAME}_PATH ${PKGPATH} PARENT_SCOPE)
endfunction()

# Sets ${PKGNAME}_FOUND
function (ros_have_pkg PKGNAME)
  ros_pkg_path(${PKGNAME})
  if (NOT "${PKGPATH}" STREQUAL "")
    set(${PKGNAME}_FOUND TRUE PARENT_SCOPE)
  endif()
endfunction()

# Sets ${PKGNAME}_MSG_CFLAGS
function (ros_pkg_msg_cflags PKGNAME)
  ros_pkg_path(${PKGNAME})
  set(${PKGNAME}_MSG_CFLAGS "-I${${PKGNAME}_PATH}/msg_gen/cpp/include -I${${PKGNAME}_PATH}/srv_gen/cpp/include" PARENT_SCOPE)
endfunction()

# Sets ${PKGNAME}_CFLAGS (includes message CFLAGS)
function (ros_pkg_cflags PKGNAME)
  execute_process(COMMAND ${ROSPACK} export --lang=cpp --attrib=cflags ${PKGNAME}
                  ERROR_QUIET OUTPUT_VARIABLE PKGCFLAGS)
  ros_pkg_msg_cflags(${PKGNAME})
  set(${PKGNAME}_CFLAGS "${${PKG_NAME}_MSG_CFLAGS} ${PKGCFLAGS}" PARENT_SCOPE)
endfunction()

# Sets ${PKGNAME}_LFLAGS
function (ros_pkg_lflags PKGNAME)
  execute_process(COMMAND ${ROSPACK} export --lang=cpp --attrib=lflags ${PKGNAME}
                  ERROR_QUIET OUTPUT_VARIABLE PKGLFLAGS)
  set(${PKGNAME}_LFLAGS ${PKGLFLAGS} PARENT_SCOPE)
endfunction()

# Sets ${PKGNAME}_CFLAGS and ${PKGNAME}_LFLAGS
function (ros_pkg_flags PKGNAME)
  ros_pkg_cflags(${PKGNAME})
  ros_pkg_lflags(${PKGNAME})
  set(${PKGNAME}_CFLAGS ${PKGNAME}_CFLAGS PARENT_SCOPE)
  set(${PKGNAME}_LFLAGS ${PKGNAME}_LFLAGS PARENT_SCOPE)
endfunction()

# Sets ${VARNAME}
function (ros_have_pkgs)
  set(VARNAME ${ARGV0})
  list(REMOVE_AT ARGV 0)
  foreach (P ${ARGV})
    ros_have_pkg(${P})
    if (NOT ${P}_FOUND)
      message(WARNING "Cannot find package ${P}")
      set(${VARNAME} FALSE PARENT_SCOPE)
    endif()
  endforeach()
  set(${VARNAME} TRUE PARENT_SCOPE)
endfunction()

# Sets ${VARNAME}
function (ros_missing_pkgs)
  set(VARNAME ${ARGV0})
  list(REMOVE_AT ARGV 0)
  set(MISSING)
  foreach (P ${ARGV})
    ros_have_pkg(${P})
    if (NOT ${P}_FOUND)
      list(APPEND MISSING ${P})
    endif()
  endforeach()
  set(${VARNAME} ${MISSING} PARENT_SCOPE)
endfunction()

# Sets ${VARNAME}
function (ros_pkgs_cflags)
  set(VARNAME ${ARGV0})
  list(REMOVE_AT ARGV 0)
  set(PKGSCFLAGS "")
  foreach (P ${ARGV})
    ros_pkg_cflags(${P})
    set(PKGSCFLAGS "${PKGSCFLAGS} ${${P}_CFLAGS}")
  endforeach()
  set(${VARNAME} ${PKGSCFLAGS} PARENT_SCOPE)
endfunction()

# Sets ${VARNAME}
function (ros_pkgs_lflags)
  set(VARNAME ${ARGV0})
  list(REMOVE_AT ARGV 0)
  set(PKGSLFLAGS "")
  foreach (P ${ARGV})
    ros_pkg_lflags(${P})
    set(PKGSLFLAGS "${PKGSLFLAGS} ${${P}_LFLAGS}")
  endforeach()
  set(${VARNAME} ${PKGSLFLAGS} PARENT_SCOPE)
endfunction()

ros_pkg_path(roscpp)
if (NOT ${roscpp_PATH} STREQUAL "")
  set(Fawkes_ros_FOUND TRUE)
  ros_pkg_flags(roscpp)
  set(Fawkes_ros_CFLAGS "-DHAVE_ROS ${roscpp_CFLAGS}")
  set(Fawkes_ros_LFLAGS "${roscpp_LFLAGS}")
endif()
