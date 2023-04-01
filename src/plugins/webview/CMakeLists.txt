set(PLUGIN_webview
    ON
    CACHE BOOL ON)

if(PLUGIN_webview)
  check_webview_deps()
  find_package(Boost COMPONENTS filesystem)
  set(SOURCES
      backendinfo-rest-api/model/Service.cpp
      backendinfo-rest-api/model/Backend.cpp
      backendinfo-rest-api/backendinfo-rest-api.cpp
      blackboard-rest-api/model/InterfaceFieldType.cpp
      blackboard-rest-api/model/InterfaceInfo.cpp
      blackboard-rest-api/model/BlackboardGraph.cpp
      blackboard-rest-api/model/InterfaceData.cpp
      blackboard-rest-api/model/InterfaceMessageType.cpp
      blackboard-rest-api/blackboard-rest-api.cpp
      plugin-rest-api/model/PluginOpResponse.cpp
      plugin-rest-api/model/PluginOpRequest.cpp
      plugin-rest-api/model/Plugin.cpp
      plugin-rest-api/plugin-rest-api.cpp
      webview_thread.cpp
      rest_processor.cpp
      static_processor.cpp
      service_browse_handler.cpp
      config-rest-api/model/ConfigTree.cpp
      config-rest-api/config-rest-api.cpp
      webview_plugin.cpp
      user_verifier.cpp)
  if(Boost_FOUND
     AND WEBVIEW_FOUND
     AND CPP_17_FOUND)
    if(TF_FOUND)
      list(APPEND SOURCES tf-rest-api/tf-rest-api.cpp
           tf-rest-api/model/TransformsGraph.cpp)
    else()
      build_skipped_message("webview transform support" "TF")
    endif()
    if(LIBJPEG_FOUND)
      list(
        APPEND SOURCES image-rest-api/model/ImageInfo.cpp
        image-rest-api/mjpeg_reply.cpp image-rest-api/jpeg_stream_producer.cpp
        image-rest-api/image-rest-api.cpp)
    else()
      build_skipped_message("webview image viewer" "libjpeg-turbo[-devel]")
    endif()
    add_subdirectory(frontend)
    add_library(webview MODULE ${SOURCES})
    target_compile_definitions(webview PUBLIC HAVE_REST_APIS)
    target_compile_features(webview PRIVATE cxx_std_17)
    if(TF_FOUMD)
      depend_on_tf(webview)
      target_link_libraries(webview fawkestf)
    endif()
    if(LIBJPEG_FOUND)
      target_compile_options(webview PUBLIC ${LIBJPEG_CFLAGS})
      target_link_libraries(webview m fvutils fvcams ${LIBJPEG_LDFLAGS})
      target_compile_definitions(webview PUBLIC HAVE_JPEG)
    endif()

    target_link_libraries(webview ${Boost_LIBRARIES})
    target_include_directories(webview PRIVATE ${Boost_INCLUDE_DIRS})
    depend_on_webview(webview)
    target_compile_definitions(webview PUBLIC HAVE_CRUIZCORE)
    target_link_libraries(
      webview
      m
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkeswebview
      fawkesplugin
      fawkesnetcomm
      fawkeslogging)
    pkg_check_modules(apr-util-1 apr-util-1)
    if(apr-util-1_FOUND)
      target_compile_options(webview PUBLIC ${apr-util-1_CFLAGS})
      target_compile_definitions(webview PUBLIC HAVE_APR_UTIL)
      target_link_libraries(webview ${apr-util-1_LDFLAGS})
    else()
      target_link_libraries(webview crypt)
      build_skipped_message("APR based password verification support"
                            "apr-util[-devel]")

    endif()
  else()
    if(NOT Boost_FOUND)
      build_skipped_message(webview "Boost filesystem")
    endif()
    if(NOT WEBVIEW_FOUND)
      build_skipped_message(webview "fawkes webview lib")
    endif()
  endif()
else()
  plugin_disabled_message("webview")
endif()
