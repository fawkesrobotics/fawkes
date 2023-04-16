set(PLUGIN_realsense2 ON CACHCE BOOL ON)
pkg_check_modules(realsense2 realsense2>=2.16.0)
remember_dependency(realsense2)

if(PLUGIN_realsense2)

  add_library(realsense2 MODULE realsense2_plugin.cpp realsense2_thread.cpp)

  depend_on_pcl(realsense2)
  depend_on_tf(realsense2)

  depend_on_pkgconfig_libs(realsense2 realsense2)

  target_compile_options(realsense2 PUBLIC ${REALSENSE2_CFLAGS})
  target_link_libraries(
    realsense2
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    fvutils
    fawkespcl_utils
    SwitchInterface
    ${REALSENSE2_LDFLAGS})
else()
  plugin_disabled_message("realsense2")
endif()
