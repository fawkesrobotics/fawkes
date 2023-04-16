set(PLUGIN_ffplugin
    ON
    CACHE BOOL ON)

if(PLUGIN_ffplugin)
  add_executable(ffplugin main.cpp plugin_tool.cpp)

  target_link_libraries(ffplugin stdc++ fawkescore fawkesutils fawkesnetcomm
                        fawkesplugin)
else()
  executable_skipped_message(ffplugin)
endif()
