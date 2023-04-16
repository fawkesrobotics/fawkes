# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann and Daniel Swoboda
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************

set(PLUGIN_webview
    ON
    CACHE BOOL ON)

if(PLUGIN_webview)
  add_library(
    webview MODULE
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

  depend_on_boost_libs(webview filesystem)

  optional_depend_on_pkgconfig_libs(webview ${tf_deps} tf_deps_found)
  if(tf_deps_found)
    target_sources(webview PRIVATE tf-rest-api/tf-rest-api.cpp
                                   tf-rest-api/model/TransformsGraph.cpp)
    depend_on_tf(webview)
    target_link_libraries(webview fawkestf)
  else()
    build_skipped_message("webview transform support" "${tf_deps}")
  endif()

  optional_depend_on_pkgconfig_libs(webview libjpeg libjpeg_dep_found)
  if(libjpeg_dep_found)
    target_sources(
      webview
      PRIVATE image-rest-api/model/ImageInfo.cpp image-rest-api/mjpeg_reply.cpp
              image-rest-api/jpeg_stream_producer.cpp
              image-rest-api/image-rest-api.cpp)
    target_compile_options(webview PUBLIC ${LIBJPEG_CFLAGS})
    target_link_libraries(webview m fvutils fvcams ${LIBJPEG_LDFLAGS})
    target_compile_definitions(webview PUBLIC HAVE_JPEG)
  else()
    build_skipped_message("webview image viewer" "libjpeg-turbo[-devel]")
  endif()
  add_subdirectory(frontend)
  target_compile_definitions(webview PUBLIC HAVE_REST_APIS)
  target_compile_features(webview PRIVATE cxx_std_17)

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
  optional_depend_on_pkgconfig_libs(webview apr-util-1 apr-util-1_dep_found)
  if(apr-util-1_dep_found)
    target_compile_options(webview PUBLIC ${apr-util-1_CFLAGS})
    target_compile_definitions(webview PUBLIC HAVE_APR_UTIL)
    target_link_libraries(webview ${apr-util-1_LDFLAGS})
  else()
    target_link_libraries(webview crypt)
    build_skipped_message("APR based password verification support"
                          "apr-util[-devel]")

  endif()
else()
  plugin_disabled_message("webview")
endif()
