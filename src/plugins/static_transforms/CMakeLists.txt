set(PLUGIN_static-transforms
    ON
    CACHE BOOL ON)

if(PLUGIN_static-transforms)

  add_library(static-transforms MODULE static_transforms_plugin.cpp
                                       static_transforms_thread.cpp)
  depend_on_tf(static-transforms)

  target_link_libraries(
    static-transforms
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    fawkestf
    fawkesconfig)

else()
  plugin_disabled_message("static-transforms")
endif()
