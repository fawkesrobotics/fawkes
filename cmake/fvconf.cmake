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

if(EXISTS "${CMAKE_SYSROOT}/usr/include/linux/videodev.h")
  set(V4L1_CAM_FOUND
      ON
      CACHE BOOL "v4l 1 support")
endif()
if(EXISTS "${CMAKE_SYSROOT}/usr/include/linux/videodev2.h")
  set(V4L2_CAM_FOUND
      ON
      CACHE BOOL "v4l 2 support")
endif()

# This replaces the VISION_CFLAGS and VISION_LDFLAGS
function(depend_on_fvconf target)
  # TODO FVCONFDIR was $(EXEC_CONFDIR)/firevision
  target_compile_options(
    ${target}
    PUBLIC -DHAVE_RECTINFO -DHAVE_NETWORK_CAM -DHAVE_FILELOADER_CAM
           -DHABE_SHMEM_CAM -D__STDC_LIMIT_MACROS
           -DFVCONFDIR="${PROJECT_SOURCE_DIR}/cfg/firevision")
  optional_depend_on_pkgconfig_libs(${target} libjpeg LIBJPEG_DEP_FOUND)
  optional_depend_on_pkgconfig_libs(${target} libdc1394-2 LIBC1394_DEP_FOUND)
  optional_depend_on_pkgconfig_libs(${target} sdl SDL_DEP_FOUND)
  optional_depend_on_pkgconfig_libs(${target} libpng LIBPNG_DEP_FOUND)
  optional_depend_on_pkgconfig_libs(${target} libv4l2 LIBV4L2_DEP_FOUND)
  optional_depend_on_pkgconfig_libs(${target} libusb LIBUSB_DEP_FOUND)
  optional_depend_on_pkgconfig_libs(${target} opencv OPENCV_DEP_FOUND)
  if(LIBUSB_DEP_FOUND)
    target_compile_definitions(${target} PUBLIC HAVE_LIBUSB)
  else()
    build_skipped_message("${target} usb features" "libusb-compat-0.1[-devel]")
  endif()

  if(OPENCV_DEP_FOUND)
    target_compile_definitions(${target} PUBLIC HAVE_OPENCV)
    target_compile_options(${target} PUBLIC -Wno-deprecated-declarations)
    set(OPENCV_FOUND
        1
        PARENT_SCOPE)
    # $(CFLAG_W_NO_UNUSED_LOCAL_TYPEDEFS)
  else()
    build_skipped_message("${target} opencv features" "opencv[-devel]")
  endif()

  if(V4L1_CAM_FOUND)
    target_compile_definitions(${target} PUBLIC HAVE_V4L1_CAM)
  endif()
  if(V4L2_CAM_FOUND)
    target_compile_definitions(${target} PUBLIC HAVE_V4L2_CAM)
  endif()

  if(LIBV4L2_DEP_FOUND)
    target_compile_definitions(${target} PUBLIC HAVE_LIBV4L2)
  else()
    build_skipped_message("${target} video4linux support" "libv4l[-devel]")
  endif()

  if(LIBPNG_DEP_FOUND)
    target_compile_definitions(${target} PUBLIC HAVE_LIBPNG)
  else()
    build_skipped_message("${target} png support" "libpng[-devel]")
  endif()

  if(SDL_DEP_FOUND)
    target_compile_definitions(${target} PUBLIC HAVE_SDL)
  else()
    build_skipped_message("${target} sdl support" "sdl12-compat[-devel]")
  endif()

  if(LIBC1394_DEP_FOUND)
    set(FIREWIRE_CAM_FOUND
        1
        PARENT_SCOPE)
    target_compile_definitions(
      ${target} PUBLIC HAVE_FIREWIRE_CAM HAVE_BUMBLEBEE2_CAM HAVE_PIKE_CAM)
  else()
    build_skipped_message("${target} firewire, bumblebee and pike support"
                          "libdc1394[-devel]")
  endif()
  if(LIBJPEG_DEP_FOUND)
    target_compile_definitions(${target} PUBLIC HAVE_LIBJPEG)
  else()
    build_skipped_message("${target} jpeg support"
                          "libjpeg-turbo[-devel] dependency")
  endif()
endfunction()
