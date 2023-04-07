set(PLUGIN_fv-base
    ON
    CACHE BOOL ON)

if(PLUGIN_fv-base)

  add_library(fv-base MODULE acquisition_thread.cpp aqt_vision_threads.cpp
                             aqt_vision_threads.cpp aqt_vision_threads.cpp)

  depend_on_fvconf(fv-base)

  target_link_libraries(
    fv-base
    fawkescore
    fawkesutils
    fawkesaspects
    fvutils
    fvcams
    fawkesblackboard
    fawkesinterface
    SwitchInterface)

else()
  plugin_disabled_message(fv-base)
endif()
