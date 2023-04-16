set(PLUGIN_pddl-planner
    ON
    CACHE BOOL ON)

add_subdirectory(interfaces)

if(PLUGIN_pddl-planner)
  add_library(pddl-planner MODULE pddl-planner_plugin.cpp
                                  pddl-planner_thread.cpp)
  depend_on_mongodb(pddl-planner)
  depend_on_pkgconfig_libs(pddl-planner libctemplate)
  depend_on_boost_libs(pddl-planner "system;filesystem")

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
    PddlPlannerInterface)

  target_compile_features(pddl-planner PUBLIC cxx_std_17)
else()
  plugin_disabled_message("pddl-planner")
endif()
