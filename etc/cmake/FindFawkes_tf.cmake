# - Fawkes: check for tf
#
# Once done this will define
#
#  Fawkes_tf_FOUND - component was found
#  Fawkes_tf_CFLAGS - Flags to add to get tf support
#  Fawkes_tf_LIBRARIES - Libs required for tf support
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

find_package(Fawkes_ros)
find_package(Fawkes_CPP11)
find_library(Fawkes_tf_LIBRARIES fawkestf HINTS "${Fawkes_LIBDIR}")

# if we can find a ROS bullet version we use that version. It is patched
# and required for the ROS integration. When mixing the system and ROS
# bullet version we will get a segfault when shutting down fawkes, always.
if (Fawkes_ros_FOUND)
  ros_have_pkg("bullet")
  if (bullet_FOUND)
    ros_pkg_flags("bullet")
    set(bullet_CFLAGS "${bullet_CFLAGS} -Wno-deprecated-declarations")
  endif()
endif()

if (NOT bullet_FOUND)
  find_package("Bullet" REQUIRED)
  set(bullet_CFLAGS "-DBT_INFINITY")
  set(bullet_INCLUDE_DIRS ${BULLET_INCLUDE_DIRS})
endif()

if (Fawkes_tf_LIBRARIES)
  set(Fawkes_tf_FOUND TRUE)
  set(Fawkes_tf_CFLAGS "${Fawkes_CPP11_CFLAGS} -DHAVE_TF ${bullet_CFLAGS} -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX")
  set(Fawkes_tf_LIBRARIES fawkestf m ${BULLET_MATH_LIBRARY})
  list(APPEND Fawkes_tf_INCLUDE_DIRS ${BULLET_INCLUDE_DIRS})
endif()
