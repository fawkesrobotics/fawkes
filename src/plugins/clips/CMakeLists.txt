set(PLUGIN_clips CACHE BOOL ON)

if(PLUGIN_clips)
  add_subdirectory(aspect)
  add_subdirectory(rest-api)

  add_library(
    clips MODULE clips_plugin.cpp clips_thread.cpp feature_blackboard.cpp
                 feature_config.cpp feature_redefine_warning.cpp)

  depend_on_clipsmm(clips)

  include_directories(aspect)
  target_link_libraries(
    clips
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesclipsaspect)
else()
  message("Skip building clips plugin")
endif()
