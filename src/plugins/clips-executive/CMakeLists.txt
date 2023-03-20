set(PLUGIN_clips-executive
    ON
    CACHE BOOL ON)

if(PLUGIN_clips-executive)
  add_subdirectory(rest-api)

  add_library(clips-executive MODULE clips_executive_plugin.cpp
                                     clips_executive_thread.cpp)

  depend_on_clipsmm(clips-executive)

  target_link_libraries(
    clips-executive
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesclipsaspect)

else()
  message("Skip building clips-executive plugin")
endif()
