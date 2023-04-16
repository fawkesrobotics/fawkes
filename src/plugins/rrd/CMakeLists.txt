set(PLUGIN_rrd
    ON
    CACHE BOOL ON)
add_subdirectory(aspect)

if(PLUGIN_rrd)
  add_library(rrd MODULE rrd_plugin.cpp rrd_thread.cpp)
  depend_on_pkgconfig_libs(rrd librrd)
  target_compile_definitions(rrd PUBLIC HAVE_RRD)
  target_link_libraries(rrd m fawkescore fawkesaspects fawkesutils
                        fawkesrrdaspect)
else()
  plugin_disabled_message("rrd")
endif()
