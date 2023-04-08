set(PLUGIN_fvbase
    ON
    CACHE BOOL ON)

if(PLUGIN_fvbase)

  add_library(fvbase MODULE fvbase_plugin.cpp base_thread.cpp
                             acquisition_thread.cpp aqt_vision_threads.cpp)

  depend_on_fvconf(fvbase)

  target_link_libraries(
    fvbase
    fawkescore
    fawkesutils
    fawkesaspects
    fvutils
    fvcams
    fawkesblackboard
    fawkesinterface
    SwitchInterface)

else()
  plugin_disabled_message(fvbase)
endif()
