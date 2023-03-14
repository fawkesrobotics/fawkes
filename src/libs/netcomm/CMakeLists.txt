include_directories(libs)
set(fawkesnetcomm_SOURCES)
list(
  APPEND
  fawkesnetcomm_SOURCES
  service_discovery/dummy_service_browser.cpp
  service_discovery/service_browser.cpp
  service_discovery/service.cpp
  service_discovery/service_publisher.cpp
  service_discovery/dummy_service_publisher.cpp
  # qa/qa_socket_datagram.cpp qa/qa_dynamic_buffer.cpp
  # qa/qa_socket_datagram_multicast.cpp qa/qa_socket_datagram_broadcast.cpp
  # qa/qa_avahi_resolver.cpp qa/qa_socket_typeof.cpp qa/qa_worldinfo.cpp
  # qa/qa_worldinfo_msgsizes.cpp qa/qa_resolver.cpp qa/qa_avahi_browser.cpp
  # qa/qa_avahi_publisher.cpp qa/qa_socket_stream.cpp
  # qa/qa_worldinfo_encryption.cpp
  utils/incoming_connection_handler.cpp
  utils/string_content.cpp
  utils/resolver.cpp
  utils/acceptor_thread.cpp
  utils/dynamic_buffer.cpp
  utils/resolver_thread.cpp
  utils/exceptions.cpp
  socket/datagram.cpp
  socket/datagram_multicast.cpp
  socket/stream.cpp
  socket/socket.cpp
  socket/datagram_broadcast.cpp
  fawkes/hub.cpp
  fawkes/network_manager.cpp
  fawkes/client.cpp
  fawkes/transceiver.cpp
  fawkes/client_handler.cpp
  fawkes/message.cpp
  fawkes/server_client_thread.cpp
  fawkes/server_thread.cpp
  fawkes/message_content.cpp
  fawkes/message_queue.cpp
  fawkes/handler.cpp
  crypto/encrypt.cpp
  crypto/decrypt.cpp)

find_package(PkgConfig REQUIRED)

pkg_check_modules(avahi-client avahi-client)
if(${avahi-client_FOUND})
  list(APPEND fawkesnetcomm_SOURCES dns-sd/avahi_thread.cpp
       dns-sd/avahi_resolver_handler.cpp)
  add_library(fawkesnetcomm SHARED ${fawkesnetcomm_SOURCES})
  target_compile_options(fawkesnetcomm PUBLIC ${avahi-client_CFLAGS}
                                              -DHAVE_AVAHI)
  target_link_libraries(fawkesnetcomm ${avahi-client_LDFLAGS})
else()
  add_library(fawkesnetcomm SHARED ${fawkesnetcomm_SOURCES})
  message("Omitting mDNS/DNS-SD support (Avahi not installed)")
endif()

include(libcrypto)
depend_on_libcrypto(fawkesnetcomm)
target_link_libraries(fawkesnetcomm stdc++ m fawkescore fawkesutils)
target_include_directories(fawkesnetcomm PUBLIC netcomm)
