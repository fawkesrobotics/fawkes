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

include_directories(libs)
add_library(
  fawkesnetcomm SHARED
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

optional_depend_on_pkgconfig_libs(fawkesnetcomm avahi-client avahi_dep_found)
if(avahi_dep_found)
  target_sources(fawkesnetcomm PRIVATE dns-sd/avahi_thread.cpp
                                       dns-sd/avahi_resolver_handler.cpp)
  target_compile_definitions(fawkesnetcomm PUBLIC HAVE_AVAHI)
else()
  build_skipped_message(fawkesnetcomm
                        "Omitting mDNS/DNS-SD support (Avahi not installed)")
endif()

depend_on_libcrypto(fawkesnetcomm)
target_link_libraries(fawkesnetcomm stdc++ m fawkescore fawkesutils)
target_include_directories(fawkesnetcomm PUBLIC netcomm)
