set(PLUGIN_robot-memory
    ON
    CACHE BOOL ON)

add_subdirectory(interfaces)
add_subdirectory(aspect)

if(PLUGIN_robot-memory)
  add_library(
    robot-memory MODULE
    robot_memory.cpp
    computables/blackboard_computable.cpp
    computables/transform_computable.cpp
    computables/computable.cpp
    computables/computables_manager.cpp
    event_trigger.cpp
    robot_memory_thread.cpp
    event_trigger_manager.cpp
    robot_memory_plugin.cpp)
  depend_on_robot_memory(robot-memory)

  target_link_libraries(
    robot-memory
    fawkescore
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesutils
    fawkeslogging
    fawkesmongodbaspect
    fvutils
    fawkestf
    RobotMemoryInterface
    fawkesrobotmemory)
  target_compile_features(robot-memory PUBLIC cxx_std_17)
else()
  plugin_disabled_message("robot-memory")
endif()
