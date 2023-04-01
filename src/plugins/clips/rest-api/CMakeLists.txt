set(PLUGIN_clips-rest-api
    ON
    CACHE BOOL ON)
if(PLUGIN_clips-rest-api)
  if(CPP_17_FOUND)
    if(WEBVIEW_FOUND)
      add_library(
        clips-rest-api MODULE
        clips-rest-api.cpp clips-rest-api-plugin.cpp model/Environment.cpp
        model/Fact.cpp model/SlotValue.cpp)

      depend_on_webview(clips-rest-api)

      target_link_libraries(clips-rest-api fawkescore fawkesutils fawkesaspects
                            fawkesclipsaspect fawkeswebview)
      target_compile_features(clips-rest-api PRIVATE cxx_std_17)
    else()
      message("Omitting clips-rest-api (webview required)")
    endif()
  else()
    message("Omitting clips-rest-api (C++ 17 required)")
  endif()
else()
  plugin_disabled_message(clips-rest-api)
endif()
