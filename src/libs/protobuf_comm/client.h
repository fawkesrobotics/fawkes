
/***************************************************************************
 *  client.h - Protobuf stream protocol - client
 *
 *  Created: Thu Jan 31 17:28:09 2013
 *  Copyright  2013  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the authors nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PROTOBUF_COMM_CLIENT_H_
#define __PROTOBUF_COMM_CLIENT_H_

#include <protobuf_comm/frame_header.h>
#include <protobuf_comm/message_register.h>
#include <protobuf_comm/queue_entry.h>

#include <boost/asio.hpp>
#include <boost/signals2.hpp>
#include <google/protobuf/message.h>
#include <queue>
#include <string>
#include <mutex>
#include <thread>
#include <cstdint>

namespace protobuf_comm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


class ProtobufStreamClient
{
 public:
  ProtobufStreamClient();
  ProtobufStreamClient(std::vector<std::string> &proto_path);
  ProtobufStreamClient(MessageRegister *mr, frame_header_version_t header_version = PB_FRAME_V2);
  ~ProtobufStreamClient();

  /** Get the client's message register.
   * @return message register
   */
  MessageRegister &  message_register()
  { return *message_register_; }

  void async_connect(const char *host, unsigned short port);
  void disconnect();

  /** Check if client is connected.
   * @return true if the client is connected, false otherwise
   */
  bool connected() const
  { return connected_; }

  void send(uint16_t component_id, uint16_t msg_type,
	    google::protobuf::Message &m);

  void send(std::shared_ptr<google::protobuf::Message> m);
  void send(google::protobuf::Message &m);

  /** Signal that is invoked when a message has been received.
   * @return signal
   */
  boost::signals2::signal<void (uint16_t, uint16_t,
				std::shared_ptr<google::protobuf::Message>)> &
    signal_received() { return sig_rcvd_; }

  /** Signal that is invoked when receiving a message failed.
   * @return signal
   */
  boost::signals2::signal<void (uint16_t, uint16_t, std::string)> &
    signal_receive_failed() { return sig_recv_failed_; }

  /** Signal that is invoked when the connection has been established.
   * @return signal
   */
  boost::signals2::signal<void ()> &
    signal_connected() { return sig_connected_; }

  /** Signal that is invoked when the connection is closed.
   * @return signal
   */
  boost::signals2::signal<void (const boost::system::error_code &)> &
    signal_disconnected() { return sig_disconnected_; }

 private: // types

 private: // methods
  void disconnect_nosig();
  void run_asio();
  void handle_resolve(const boost::system::error_code& err,
		      boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
  void handle_connect(const boost::system::error_code& err);
  void handle_write(const boost::system::error_code& error,
		    size_t /*bytes_transferred*/, QueueEntry *entry);
  void start_recv();
  void handle_read_header(const boost::system::error_code& error);
  void handle_read_message(const boost::system::error_code& error);

 private: // members
  bool connected_;
  std::mutex                      asio_mutex_;
  boost::asio::io_service         io_service_;
  boost::asio::ip::tcp::resolver  resolver_;
  boost::asio::ip::tcp::socket    socket_;
  boost::asio::io_service::work   io_service_work_;

  boost::signals2::signal<void (uint16_t, uint16_t,
				std::shared_ptr<google::protobuf::Message>)>  sig_rcvd_;
  boost::signals2::signal<void (uint16_t, uint16_t, std::string)> sig_recv_failed_;
  boost::signals2::signal<void ()> sig_connected_;
  boost::signals2::signal<void (const boost::system::error_code &)> sig_disconnected_;

  std::thread asio_thread_;

  std::queue<QueueEntry *> outbound_queue_;
  std::mutex               outbound_mutex_;
  bool                     outbound_active_;

  void   *in_frame_header_;
  size_t  in_frame_header_size_;
  size_t  in_data_size_;
  void *  in_data_;

  MessageRegister *message_register_;
  bool             own_message_register_;

  frame_header_version_t frame_header_version_;
};

} // end namespace protobuf_comm

#endif
