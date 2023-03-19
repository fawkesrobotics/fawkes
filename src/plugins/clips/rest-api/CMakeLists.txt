set(PLUGIN_clips-rest-api CACHE BOOL ON)
if(PLUGIN_clips-rest-api)
  list(FIND CMAKE_CXX_COMPILE_FEATURES "cxx_std_20" _index)
  if(${_index} GREATER -1)
    add_library(clips-rest-api MODULE clips-rest-api.cpp
                                      clips-rest-api-plugin.cpp)

    depend_on_webview(clips-rest-api)

    target_link_libraries(clips-rest-api fawkescore fawkesutils fawkesaspects
                          fawkesclipsaspect fawkeswebview)
    target_compile_features(clips-rest-api PRIVATE cxx_std_17)
  else()
    message("Omitting clips-rest-api (C++ 17 required)")
  endif()
else()
  message("Skip building clips-rest-api plugin")
endif()
