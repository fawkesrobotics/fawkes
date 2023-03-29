set(PLUGIN_rrd
    ON
    CACHE BOOL ON)
add_subdirectory(aspect)
pkg_check_modules(rrd librrd)

if(PLUGIN_rrd)
  if(rrd_FOUND)
    add_library(rrd MODULE rrd_plugin.cpp rrd_thread.cpp)
    target_link_libraries(rrd ${rrd_LDFLAGS})
    target_compile_definitions(rrd PUBLIC HAVE_RRD)
    target_compile_options(rrd PUBLIC ${rrd_CFLAGS})
    target_link_libraries(rrd m fawkescore fawkesaspects fawkesutils
                          fawkesrrdaspect)
  else()
    build_skipped_message("rrd" "rrdtool-[devel]")
  endif()
else()
  plugin_disabled_message("rrd")
endif()
