set(PLUGIN_pddl-planner
    ON
    CACHE BOOL ON)

find_package(Boost COMPONENTS system filesystem)

add_subdirectory(interfaces)

if(PLUGIN_pddl-planner)
  pkg_check_modules(ctemplate libctemplate)
  if(ctemplate_FOUND)
    if(mongocxx_FOUND)
      if(CPP_17_FOUND)
        add_library(pddl-planner MODULE pddl-planner_plugin.cpp
                                        pddl-planner_thread.cpp)
        depend_on_mongodb(pddl-planner)
        target_link_libraries(
          pddl-planner
          m
          fawkescore
          fawkesutils
          fawkesaspects
          fawkesbaseapp
          fawkesblackboard
          fawkesinterface
          fawkesrobotmemory
          PddlPlannerInterface
          ${ctemplate_LDFLAGS})

        target_compile_options(pddl-planner PUBLIC ${ctemplate_CFLAGS})
        target_compile_features(pddl-planner PUBLIC cxx_std_17)

      else()
        build_skipped_message(pddl-planner "C++ 17")
      endif()
    else()
      build_skipped_message(pddl-planner "mongo-cxx-driver[-devel]")
    endif()
  else()
    build_skipped_message(pddl-robot-memory "ctemplate[-devel]")
  endif()
else()
  plugin_disabled_message("pddl-planner")
endif()
