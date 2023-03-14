add_library(fawkesplugin SHARED net/list_message.cpp net/handler.cpp
                                listener.cpp manager.cpp loader.cpp)

pkg_check_modules(libelf REQUIRED libelf)
target_compile_options(fawkesconfig PUBLIC -DHAVE_LIBELF)

if(EXISTS "/usr/include/sys/inotify.h")
  target_compile_options(fawkesconfig PUBLIC -DHAVE_INOTIFY)
endif()

target_link_libraries(
  fawkesplugin
  stdc++
  elf
  fawkescore
  fawkesutils
  fawkesconfig
  fawkesnetcomm
  fawkeslogging
  dl)
