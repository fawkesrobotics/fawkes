set(PLUGIN_skiller-simulator
    ON
    CACHE BOOL ON)

if(PLUGIN_skiller-simulator)
  if(CPP_17_FOUND)
    add_library(skiller-simulator MODULE exec_thread.cpp
                                         skiller_simulator_plugin.cpp)

    target_link_libraries(
      skiller-simulator
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkeslogging
      fawkes_execution_time_estimator
      fawkes_execution_time_estimator_aspect)
  else()
    build_skipped_message("skiller-simulator" "CPP17")
  endif()
else()
  plugin_disabled_message("skiller-simulator")
endif()
