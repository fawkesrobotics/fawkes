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
  optional_depend_on_pkgconfig_libs(${target} libjpeg libjpeg_dep_found)
  optional_depend_on_pkgconfig_libs(${target} libdc1394-2 libdc1394_dep_found)
  optional_depend_on_pkgconfig_libs(${target} sdl sdl_dep_found)
  optional_depend_on_pkgconfig_libs(${target} libpng libpng_dep_found)
  optional_depend_on_pkgconfig_libs(${target} libv4l2 libv4l2_dep_found)
  optional_depend_on_pkgconfig_libs(${target} libusb libusb_dep_found)
  optional_depend_on_pkgconfig_libs(${target} opencv opencv_dep_found)
  if(libusb_dep_found)
    target_compile_definitions(${target} PUBLIC HAVE_LIBUSB)
  else()
    build_skipped_message("${target} usb features" "libusb-compat-0.1[-devel]")
  endif()

  if(opencv_dep_found)
    target_compile_definitions(${target} PUBLIC HAVE_OPENCV)
    target_compile_options(${target} PUBLIC -Wno-deprecated-declarations)
    set(OPENCV_FOUND
        1
        PARENT_SCOPE)
    # $(CFLAG_W_NO_UNUSED_LOCAL_TYPEDEFS)
  else()
    build_skipped_message("${target} opencv features" "opencv[-devel]")
  endif()

  if(libv4l2_dep_found)
    target_compile_definitions(${target} PUBLIC HAVE_LIBV4L2)
  else()
    build_skipped_message("${target} video4linux support" "libv4l[-devel]")
  endif()

  if(V4L1_CAM_FOUND)
    target_compile_definitions(${target} PUBLIC HAVE_V4L1_CAM)
  endif()
  if(V4L2_CAM_FOUND)
    target_compile_definitions(${target} PUBLIC HAVE_V4L2_CAM)
  endif()

  if(libpng_dep_found)
    target_compile_definitions(${target} PUBLIC HAVE_LIBPNG)
  else()
    build_skipped_message("${target} png support" "libpng[-devel]")
  endif()

  if(sdl_dep_found)
    target_compile_definitions(${target} PUBLIC HAVE_SDL)
  else()
    build_skipped_message("${target} sdl support" "sdl12-compat[-devel]")
  endif()

  if(libdc1394_dep_found)
    target_compile_definitions(
      ${target} PUBLIC HAVE_FIREWIRE_CAM HAVE_BUMBLEBEE2_CAM HAVE_PIKE_CAM)
  else()
    build_skipped_message("${target} firewire, bumblebee and pike support"
                          "libdc1394[-devel]")
  endif()
  if(libjpeg_dep_found)
    target_compile_definitions(${target} PUBLIC HAVE_LIBJPEG)
  else()
    build_skipped_message("${target} jpeg support"
                          "libjpeg-turbo[-devel] dependency")
  endif()
endfunction()
