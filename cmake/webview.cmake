set(webview_deps "libmicrohttpd;RapidJSON")

function(depend_on_webview target)
  target_link_libraries(${target} ${libmicrohttpd_LDFLAGS} ${RAPIDJOSN_LDFLAGS})
  target_compile_options(${target} PRIVATE ${libmicrohttpd_CFLAGS}
                                           ${RAPIDJOSN_CFLAGS})
  target_compile_definitions(${target} PUBLIC HAVE_WEBVIEW)
endfunction()
