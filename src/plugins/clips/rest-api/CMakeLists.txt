set(PLUGIN_clips-rest-api
    ON
    CACHE BOOL ON)
if(PLUGIN_clips-rest-api)
  add_library(
    clips-rest-api MODULE
    clips-rest-api.cpp clips-rest-api-plugin.cpp model/Environment.cpp
    model/Fact.cpp model/SlotValue.cpp)

  depend_on_webview(clips-rest-api)

  target_link_libraries(clips-rest-api fawkescore fawkesutils fawkesaspects
                        fawkesclipsaspect fawkeswebview)
  target_compile_features(clips-rest-api PRIVATE cxx_std_17)
else()
  plugin_disabled_message(clips-rest-api)
endif()
