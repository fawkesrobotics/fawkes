set(PLUGIN_fvfountain
    ON
    CACHE BOOL ON)

if(PLUGIN_fvfountain)
  add_library(fvfountain MODULE fountain_plugin.cpp fountain_thread.cpp)
  # mini_image_producer.cpp is outdated code, excluded here  

  depend_on_fvconf(fvfountain)

  target_link_libraries(fvfountain fawkescore fawkesutils fvutils fawkesnetcomm
                        fawkesaspects fawkeslogging)

else()
  plugin_disabled_message(fvfountain)
endif()
