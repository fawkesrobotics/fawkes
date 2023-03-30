set(PLUGIN_clips-tf
    ON
    CACHE BOOL ON)

if(PLUGIN_clips-tf)
  if(CPP_11_FOUND)
    add_library(clips-tf MODULE clips_tf_plugin.cpp clips_tf_thread.cpp)
    depend_on_clipsmm(clips-tf)
    depend_on_tf(clips-tf)
    target_link_libraries(clips-tf fawkescore fawkesutils fawkesaspects
                          fawkesclipsaspect fawkestf)
  else()
    build_skipped_message("clips-tf" "CPP11")
  endif()
else()
  plugin_disabled_message("clips-tf")
endif()
