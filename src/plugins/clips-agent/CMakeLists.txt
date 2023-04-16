set(PLUGIN_clips-agent
    ON
    CACHE BOOL ON)

if(PLUGIN_clips-agent)

  add_library(clips-agent MODULE clips_agent_plugin.cpp clips_agent_thread.cpp)

  depend_on_clipsmm(clips-agent)

  target_link_libraries(
    clips-agent
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesclipsaspect
    SkillerInterface
    SwitchInterface)

  # install(FILES clips/init.clp clips/skills.clp)

else()
  plugin_disabled_message(clips-agent)
endif()
