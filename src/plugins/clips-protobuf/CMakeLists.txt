set(PLUGIN_clips-protobuf
    ON
    CACHE BOOL ON)

if(PLUGIN_clips-protobuf)
  add_library(clips-protobuf MODULE clips-protobuf-plugin.cpp
                                    clips-protobuf-thread.cpp)
  target_compile_features(clips-protobuf PUBLIC cxx_std_11)
  depend_on_clipsmm(clips-protobuf)
  depend_on_protobuf(clips-protobuf)
  depend_on_boost_libs(clips-protobuf system)
  target_link_libraries(clips-protobuf fawkescore fawkesutils fawkesaspects
                        fawkesclipsaspect fawkes_protobuf_clips)

else()
  plugin_disabled_message("clips-protobuf")
endif()
