set(PLUGIN_clips-executive-rest-api CACHE BOOL ON)
if(PLUGIN_clips-executive-rest-api)
  list(FIND CMAKE_CXX_COMPILE_FEATURES "cxx_std_17" _index)
  if(${_index} GREATER -1)
    add_library(
      clips-executive-rest-api MODULE clips-executive-rest-api.cpp
                                      clips-executive-rest-api-plugin.cpp)

    depend_on_webview(clips-executive-rest-api)

    target_link_libraries(clips-executive-rest-api fawkescore fawkesutils
                          fawkesaspects fawkesclipsaspect fawkeswebview)
    target_compile_features(clips-executive-rest-api PRIVATE cxx_std_17)
  else()
    message("Omitting clips-executive-rest-api (C++ 17 required)")
  endif()
else()
  message("Skip building clips-executive-rest-api plugin")
endif()
