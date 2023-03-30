set(PLUGIN_clips-protobuf
    ON
    CACHE BOOL ON)

if(PLUGIN_clips-protobuf)
  if(CPP_11_FOUND)
    add_library(clips-protobuf MODULE clips-protobuf-plugin.cpp
                                      clips-protobuf-thread.cpp)
    depend_on_clipsmm(clips-protobuf)
    depend_on_protobuf(clips-protobuf)
    find_package(Boost COMPONENTS system)
    if(Boost_FOUND)
      target_link_libraries(
        clips-protobuf
        fawkescore
        fawkesutils
        fawkesaspects
        fawkesclipsaspect
        fawkes_protobuf_clips
        ${Boost_LIBRARIES})
      target_include_directories(clips-protobuf PUBLIC ${Boost_INCLUDE_DIRS})
    else()
      build_skipped_message("clips-protobuf" "Boost System")
    endif()
  else()
    build_skipped_message("clips-protobuf" "CPP11")
  endif()

else()
  plugin_disabled_message("clips-protobuf")
endif()
