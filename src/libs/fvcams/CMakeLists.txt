set(fvcams_SOURCES)
list(
  APPEND
  fvcams_SOURCES
  # kinect.cpp # was never built?
  factory.cpp
  # qa/qa_fwcam.cpp
  control/factory.cpp
  control/zoom.cpp
  # control/sony_evid100p.cpp # outdated
  control/pantilt.cpp
  control/focus.cpp
  # control/visca.cpp # outdated
  control/color.cpp
  control/image.cpp
  control/effect.cpp
  control/control.cpp
  control/source.cpp
  control/dummy.cpp
  buffer.cpp
  cam_exceptions.cpp
  camera.cpp
  # leutron.cpp # outdated nao.cpp # outdated v4l1.cpp # outdated
  net.cpp
  fileloader.cpp
  shmem.cpp)

if(LIBDC1394_FOUND)
  list(APPEND fvcams_SOURCES firewire.cpp bumblebee2.cpp pike.cpp)
else()
  message(
    WARNING
      "Omitting firewire bumblebee3 and pike cams (libdc1394[-devel] not installed)"
  )
endif()

if(LIBV4L2_FOUND)
  list(APPEND fvcams_SOURCES v4l2.cpp v4l.cpp)
else()
  message(WARNING "Omitting V4l2 support (libv4l[-devel] not installed)")
endif()

add_library(fvcams SHARED ${fvcams_SOURCES})
depend_on_fvconf(fvcams)
target_link_libraries(fvcams fawkescore fawkesutils fvutils fawkeslogging)
