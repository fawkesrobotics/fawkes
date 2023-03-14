pkg_check_modules(LIBMICROHTTPD libmicrohttpd)
pkg_check_modules(RAPIDJSON RapidJSON)

function(check_webview_deps WEBVIEW_FOUND)
  if(LIBMICROHTTPD_FOUND AND RAPIDJSON_FOUND)
    set(WEBVIEW_FOUND
        1
        PARENT_SCOPE)
  else()
    set(WEBVIEW_FOUND
        0
        PARENT_SCOPE)
    if(NOT LIBMICROHTTPD_FOUND)
      message(WARNING "libmicrohttpd dependency missing")
    endif()
    if(NOT RAPIDJSON_FOUND)
      message(WARNING "rapidjson[-devel] dependency missing")
    endif()
  endif()
endfunction()

function(depend_on_webview target)
  target_link_libraries(${target} ${LIBMICROHTTPD_LDFLAGS} ${RAPIDJOSN_LDFLAGS})
  target_compile_options(${target} PRIVATE ${LIBMICROHTTPD_CFLAGS}
                                           ${RAPIDJOSN_CFLAGS})
endfunction()
