set(PLUGIN_pddl-robot-memory
    ON
    CACHE BOOL ON)

find_package(Boost COMPONENTS system filesystem)

add_subdirectory(interfaces)

if(PLUGIN_pddl-robot-memory)
  pkg_check_modules(ctemplate libctemplate)
  if(ctemplate_FOUND)
    if(robot_memory_FOUND)
      if(CPP_17_FOUND)
        add_library(pddl-robot-memory MODULE pddl_robot_memory_plugin.cpp
                                             pddl_robot_memory_thread.cpp)
        depend_on_robot_memory(pddl-robot-memory)
        target_link_libraries(
          pddl-robot-memory
          m
          fawkescore
          fawkesutils
          fawkesaspects
          fawkesbaseapp
          fawkesblackboard
          fawkesinterface
          fawkesrobotmemory
          PddlGenInterface
          ${ctemplate_LDFLAGS})

        target_compile_options(pddl-robot-memory PUBLIC ${ctemplate_CFLAGS})
        target_compile_features(pddl-robot-memory PUBLIC cxx_std_17)

      else()
        build_skipped_message(pddl-robot-memory "C++ 17")
      endif()
    else()
      build_skipped_message(pddl-robot-memory "robot-memory dependencies")
    endif()
  else()
    build_skipped_message(pddl-robot-memory "ctemplate[-devel]")
  endif()

else()
  plugin_disabled_message("pddl-robot-memory")
endif()
