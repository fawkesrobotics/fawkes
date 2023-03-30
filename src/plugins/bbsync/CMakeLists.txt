set(PLUGIN_bbsync
    ON
    CACHE BOOL ON)
add_subdirectory(interfaces)
if(PLUGIN_bbsync)
  add_library(bbsync MODULE bbsync_plugin.cpp sync_listener.cpp sync_thread.cpp
                            writer_listener.cpp)

  target_link_libraries(
    bbsync
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    HeartbeatInterface)

else()
  plugin_disabled_message("bbsync")
endif()
