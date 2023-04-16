set(PLUGIN_pddl-robot-memory
    ON
    CACHE BOOL ON)

find_package(Boost COMPONENTS system filesystem)

add_subdirectory(interfaces)

if(PLUGIN_pddl-robot-memory)
  add_library(pddl-robot-memory MODULE pddl_robot_memory_plugin.cpp
                                       pddl_robot_memory_thread.cpp)
  depend_on_robot_memory(pddl-robot-memory)
  depend_on_pkgconfig_libs(pddl-robot-memory libctemplate)
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
    PddlGenInterface)

  target_compile_features(pddl-robot-memory PUBLIC cxx_std_17)

else()
  plugin_disabled_message("pddl-robot-memory")
endif()
