set(PLUGIN_clips-robot-memory
    ON
    CACHE BOOL ON)

if(PLUGIN_clips-robot-memory)
  if(CLIPSMM_FOUND)
    if(robot_memory_FOUND)
      if(CPP_11_FOUND)
        add_library(
          clips-robot-memory MODULE
          clips_robot_memory_plugin.cpp clips_robot_memory_thread.cpp
          clips_rm_trigger.cpp)
        depend_on_clipsmm(clips-robot-memory)
        depend_on_robot_memory(clips-robot-memory)
        target_link_libraries(
          clips-robot-memory
          m
          fawkescore
          fawkesutils
          fawkesaspects
          fawkesbaseapp
          fawkesblackboard
          fawkesinterface
          fawkesclipsaspect
          fawkesrobotmemory)
        target_compile_features(clips-robot-memory PUBLIC cxx_std_11)

      else()
        build_skipped_message(clips-robot-memory "C++ 11")
      endif()
    else()
      build_skipped_message(clips-robot-memory "robot-memory dependencies")
    endif()
  else()
    build_skipped_message(clips-robot-memory "clipsmm[-devel]")
  endif()

else()
  plugin_disabled_message("clips-robot-memory")
endif()
