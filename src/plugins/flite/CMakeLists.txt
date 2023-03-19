set(PLUGIN_flite
    ON
    CACHE BOOL ON)

if(PLUGIN_flite)
  find_path(FLITE_ROOT flite/flite.h PATH_SUFFIXES include)
  pkg_check_modules(ALSA alsa)
  if(FLITE_ROOT AND ALSA_FOUND)
    add_library(FLite INTERFACE IMPORTED)
    target_include_directories(FLite INTERFACE ${FLITE_ROOT})
    target_link_directories(FLite INTERFACE ${FLITE_ROOT}/lib)
    target_link_libraries(FLite INTERFACE -lflite_cmu_us_kal -lflite_usenglish
                                          -lflite_cmulex -lflite -lm)

    add_library(flite MODULE flite_plugin.cpp synth_thread.cpp)

    target_compile_options(flite PUBLIC ${ALSA_CFLAGS})

    target_link_libraries(
      flite
      m
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      SpeechSynthInterface
      FLite
      ${ALSA_LDFLAGS})

  else()
    message(WARNING "Omitting Plugin flite")
    if(NOT FLITE_ROOT)
      message(WARNING "flite[-devel] dependency missing")
    endif()
    if(NOT ALSA_FOUND)
      message(WARNING "alsa-lib[-devel] dependency missing")
    endif()
  endif()
else()
  message(STATUS "Skip building disabled flite plugin")
endif()
