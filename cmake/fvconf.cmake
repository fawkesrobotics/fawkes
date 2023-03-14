pkg_check_modules(LIBJPEG libjpeg)
if(NOT LIBJPEG_FOUND)
  message(WARNING "libjpeg-turbo[-devel] dependency missing")
endif()

message("Checking vision-related features")
pkg_check_modules(LIBDC1394 libdc1394-2)
if(NOT LIBDC1394_FOUND)
  message(WARNING "libdc1394[-devel] dependency missing")
endif()

pkg_check_modules(SDL sdl)
if(NOT SDL_FOUND)
  message(WARNING "sdl12-compat[-devel] dependency missing")
endif()

pkg_check_modules(LIBPNG libpng)
if(NOT LIBPNG_FOUND)
  message(WARNING "libpng[-devel] dependency missing")
endif()

pkg_check_modules(LIBV4L2 libv4l2)
if(NOT LIBV4L2_FOUND)
  message(WARNING "libv4l[-devel] dependency missing")
endif()

pkg_check_modules(LIBUSB libusb)
if(NOT LIBUSB_FOUND)
  message(WARNING "libusb-compat-0.1[-devel] dependency missing")
endif()

pkg_check_modules(OPENCV opencv)
if(NOT OPENCV_FOUND)
  message(WARNING "opencv[-devel] dependency missing")
endif()

# This replaces the VISION_CFLAGS and VISION_LDFLAGS
function(depend_on_fvconf target)
  # TODO FVCONFDIR was $(EXEC_CONFDIR)/firevision
  target_compile_options(
    ${target}
    PUBLIC -DHAVE_RECTINFO -DHAVE_NETWORK_CAM -DHAVE_FILELOADER_CAM
           -DHABE_SHMEM_CAM -D__STDC_LIMIT_MACROS
           -DFVCONFDIR="${PROJECT_SOURCE_DIR}/cfg/firevision")
  if(LIBUSB_FOUND)
    target_compile_options(${target} PUBLIC -DHAVE_LIBUSB ${LIBUSB_CFLAGS})
    target_link_libraries(${target} ${LIBUSB_LDFLAGS})
  endif()
  if(OPENCV_FOUND)
    target_compile_options(${target} PUBLIC -DHAVE_OPENCV ${OPENCV_CFLAGS}
                                            -Wno-deprecated-declarations)
    # $(CFLAG_W_NO_UNUSED_LOCAL_TYPEDEFS)
    target_link_libraries(${target} ${OPENCV_LDFLAGS})
  endif()
  if(LIBV4L2_FOUND)
    target_compile_options(${target} PUBLIC -DHAVE_LIBV4L2 ${LIBV4L2_CFLAGS})
    target_link_libraries(${target} ${LIBV4L2_LDFLAGS})
  endif()
  if(LIBPNG_FOUND)
    target_compile_options(${target} PUBLIC -DHAVE_LIBPNG ${LIBPNG_CFLAGS})
    target_link_libraries(${target} ${LIBPNG_LDFLAGS})
  endif()
  if(SDL_FOUND)
    target_compile_options(${target} PUBLIC -DHAVE_SDL ${SDL_CFLAGS})
    target_link_libraries(${target} ${SDL_LDFLAGS})
  endif()
  if(LIBDC1394_FOUND)
    target_compile_options(
      ${target} PUBLIC -DHAVE_FIREWIRE_CAM -DHAVE_BUMBLEBEE2_CAM
                       -DHAVE_PIKE_CAM ${LIBDC1394_CFLAGS})
    target_link_libraries(${target} ${LIBDC1394_LDFLAGS})
  endif()
  if(LIBJPEG_FOUND)
    target_compile_options(${target} PUBLIC -DHAVE_LIBJPEG ${LIBJPEG_CFLAGS})
    target_link_libraries(${target} ${LIBJPEG_LDFLAGS})
  endif()
endfunction()
