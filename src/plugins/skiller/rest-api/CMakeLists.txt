set(PLUGIN_skiller-rest-api
    ON
    CACHE BOOL ON)
if(PLUGIN_skiller-rest-api)
  if(CPP_17_FOUND)
    if(WEBVIEW_FOUND)
      add_library(
        skiller-rest-api MODULE
        model/SkillCall.cpp model/Skill.cpp model/SkillInfo.cpp
        skiller-rest-api-plugin.cpp skiller-rest-api.cpp)

      depend_on_webview(skiller-rest-api)

      target_link_libraries(
        skiller-rest-api
        fawkescore
        fawkesutils
        fawkesaspects
        fawkeswebview
        fawkesblackboard
        SkillerDebugInterface
        SkillerInterface)
      target_compile_features(skiller-rest-api PRIVATE cxx_std_17)
    else()
      build_skipped_message(skiller-rest-api webview)
    endif()
  else()
    build_skipped_message(skiller-rest-api "C++ 17")
  endif()
else()
  plugin_disabled_message(skiller-rest-api)
endif()
