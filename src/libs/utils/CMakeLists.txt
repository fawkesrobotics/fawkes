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
  # tests/catch2_main.cpp
  # tests/test_uuid.cpp
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

set(LUA_SOURCES time/clock.tolua time/time.tolua kalman/kalman_1d.tolua)
set(LUA_LINK_LIBS fawkescore fawkesutils)
generate_lua_lib(fawkesutils "${LUA_SOURCES}" "${LUA_LINK_LIBS}")
depend_on_lua(fawkesutils)

optional_depend_on_pkgconfig_libs(fawkesutils libmagic libmagic_dep_found)
if(libmagic_dep_found)
  target_compile_definitions(fawkesutils PUBLIC HAVE_LIBMAGIC)
else()
  build_skipped_message(
    fwakesutils
    "Restricted file type detection support (file[-devel] (libmagic)) not installed"
  )
endif()

optional_depend_on_pkgconfig_libs(fawkesutils libdaemon libdaemon_dep_found)
if(NOT libdaemon_dep_found)
  build_skipped_message(
    fwakesutils
    "Sub-process cannot close all file handles (libdaemon not found)")
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
  uuid)

depend_on_boost_libs(fawkesutils system)

target_include_directories(fawkesutils PUBLIC core)
target_compile_definitions(fawkesutils PUBLIC HAVE_EXECINFO HAVE_INOTIFY)
