include_directories(libs)
add_library(
  fawkesutils SHARED
  sub_process/proc.cpp
  hungarian_method/hungarian.cpp
  misc/map_skill.cpp
  misc/string_conversions.cpp
  misc/autofree.cpp
  misc/strndup.cpp
  misc/string_compare.cpp
  misc/string_commands.cpp
  # qa/qa_liblogger.cpp
  # qa/plugin.cpp
  # qa/qa_ipc_shmem_lock.cpp
  # qa/qa_ipc_shmem_lowlevel.cpp
  # qa/qa_time.cpp
  # qa/test_plugin.cpp
  # qa/qa_angle.cpp
  # qa/qa_logger.cpp
  # qa/qa_filetype.cpp
  # qa/qa_ipc_msg.cpp
  # qa/qa_ipc_shmem.cpp
  # qa/qa_timebug.cpp
  # qa/qa_hostinfo.cpp
  # qa/qa_pathparser.cpp
  # qa/qa_hungarian.cpp
  # qa/qa_ipc_semset.cpp
  time/tracker.cpp
  time/simts.cpp
  time/time.cpp
  time/watch.cpp
  time/wait.cpp
  time/clock.cpp
  fawkesutils_tolua.cpp
  search/astar.cpp
  constraints/dependency.cpp
  uuid.cpp
  tests/catch2_main.cpp
  tests/test_uuid.cpp
  math/interpolation/interpolator.cpp
  math/interpolation/linear.cpp
  math/interpolation/sinusoidal.cpp
  ipc/shm_registry.cpp
  ipc/shm.cpp
  ipc/shm_exceptions.cpp
  ipc/semset.cpp
  ipc/msg.cpp
  system/file.cpp
  system/pathparser.cpp
  system/fam_thread.cpp
  system/getkey.cpp
  system/dynamic_module/module_manager.cpp
  system/dynamic_module/module.cpp
  system/argparser.cpp
  system/signal.cpp
  system/filetype.cpp
  system/hostinfo.cpp
  system/fam.cpp
  kalman/kalman_1d.cpp)
target_compile_features(fawkesutils PUBLIC cxx_std_11)
# TODO BOOST
find_package(Boost REQUIRED COMPONENTS system)

include(lua)
generate_tolua(fawkesutils time/clock.tolua time/time.tolua
               kalman/kalman_1d.tolua)
add_library(fawkesutils_tolua SHARED fawkesutils_tolua.cpp)
depend_on_lua(fawkesutils_tolua)
target_link_libraries(fawkesutils_tolua fawkescore fawkesutils)

pkg_check_modules(libmagic libmagic)
if(libmagic_FOUND)
  target_compile_options(fawkesutils PUBLIC -DHAVE_LIBMAGIC)
  target_compile_options(fawkesutils PUBLIC ${libmagic_CFLAGS})
  target_link_libraries(fawkesutils ${libmagic_LDFLAGS})
else()
  message(
    WARNING
      "Restricted file type detection support (file[-devel] (libmagic)) not installed"
  )
endif()
pkg_check_modules(libdaemon libdaemon)
if(libdaemon_FOUND)
  target_compile_options(fawkesutils PUBLIC ${libdaemon_CFLAGS})
  target_link_libraries(fawkesutils ${libdaemon_LDFLAGS})
else()
  message(
    WARNING "Sub-process cannot close all file handles (libdaemon not found)")
endif()
target_link_libraries(
  fawkesutils
  pthread
  stdc++
  m
  rt
  fawkescore
  fawkeslogging
  dl
  pthread
  uuid
  ${Boost_SYSTEM_LIBRARY})
target_include_directories(fawkesutils PUBLIC core)
target_compile_options(fawkesutils PUBLIC -DHAVE_EXECINFO -DHAVE_INOTIFY)
