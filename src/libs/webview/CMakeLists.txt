set(WEBVIEW_FOUND 0)
check_webview_deps(WEBVIEW_FOUND)

if(WEBVIEW_FOUND)
  add_library(
    fawkeswebview SHARED
    page_footer_generator.cpp
    nav_manager.cpp
    url_manager.cpp
    request_manager.cpp
    reply.cpp
    page_header_generator.cpp
    request.cpp
    request_dispatcher.cpp
    redirect_reply.cpp
    server.cpp
    rest_api_manager.cpp
    access_log.cpp
    rest_api.cpp
    file_reply.cpp
    error_reply.cpp
    page_reply.cpp
    user_verifier.cpp)
  depend_on_webview(fawkeswebview)
  target_link_libraries(fawkeswebview fawkescore fawkesutils fawkeslogging)
else()
  message(WARNING "Skipped due to missing dependencies")
endif()
