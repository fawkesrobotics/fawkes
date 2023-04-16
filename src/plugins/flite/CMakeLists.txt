set(PLUGIN_flite
    ON
    CACHE BOOL ON)

if(PLUGIN_flite)
  find_path(FLITE_ROOT flite/flite.h PATH_SUFFIXES include)

  add_library(flite MODULE flite_plugin.cpp synth_thread.cpp)

  depend_on_pkgconfig_libs(flite alsa)

  target_link_libraries(
    flite
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    SpeechSynthInterface
    FLite)

  if(NOT FLITE_ROOT)
    disable_taget(flite)
    build_skipped_message("flite" "flite[-devel] dependency")
  endif()
else()
  plugin_disabled_message(flite)
endif()
