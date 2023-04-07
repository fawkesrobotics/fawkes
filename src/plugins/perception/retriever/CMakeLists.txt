set(PLUGIN_fvretriever
    ON
    CACHE BOOL ON)

if(PLUGIN_fvretriever)
  add_library(fvretriever MODULE fvretriever_plugin.cpp retriever_thread.cpp)
  depend_on_fvconf(fvretriever)

  target_link_libraries(
    fvretriever
    fawkescore
    fawkesutils
    fawkesaspects
    fvutils
    fvcams
    fvmodels)

else()
  plugin_disabled_message(fvretriever)
endif()
