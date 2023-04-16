set(PLUGIN_clips-tf
    ON
    CACHE BOOL ON)

if(PLUGIN_clips-tf)
  add_library(clips-tf MODULE clips_tf_plugin.cpp clips_tf_thread.cpp)
  target_compile_features(clips-tf PUBLIC cxx_std_11)
  depend_on_clipsmm(clips-tf)
  depend_on_tf(clips-tf)
  target_link_libraries(clips-tf fawkescore fawkesutils fawkesaspects
                        fawkesclipsaspect fawkestf)
else()
  plugin_disabled_message("clips-tf")
endif()
