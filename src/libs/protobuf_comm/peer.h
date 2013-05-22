
/***************************************************************************
 *  server.h - Protobuf stream protocol - broadcast peer
 *
 *  Created: Wed Jan 30 16:41:22 2013
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

#ifndef __PROTOBUF_COMM_PEER_H_
#define __PROTOBUF_COMM_PEER_H_

#include <protobuf_comm/frame_header.h>
#include <protobuf_comm/message_register.h>
#include <protobuf_comm/queue_entry.h>

#include <boost/asio.hpp>
#include <boost/signals2.hpp>
#include <google/protobuf/message.h>

#include <thread>
#include <mutex>
#include <queue>

namespace protobuf_comm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ProtobufBroadcastPeer
{
 public:
  enum { max_packet_length = 1024 };

  ProtobufBroadcastPeer(const std::string address, unsigned short port);
  ProtobufBroadcastPeer(const std::string address, unsigned short send_to_port,
			unsigned short recv_on_port);
  ProtobufBroadcastPeer(const std::string address, unsigned short port,
			std::vector<std::string> &proto_path);
  ProtobufBroadcastPeer(const std::string address, unsigned short send_to_port,
			unsigned short recv_on_port, std::vector<std::string> &proto_path);
  ProtobufBroadcastPeer(const std::string address, unsigned short port, MessageRegister *mr);
  ProtobufBroadcastPeer(const std::string address, unsigned short send_to_port,
			unsigned short recv_on_port, MessageRegister *mr);
  ~ProtobufBroadcastPeer();

  void set_filter_self(bool filter);

  void send(uint16_t component_id, uint16_t msg_type,
	    google::protobuf::Message &m);
  void send(uint16_t component_id, uint16_t msg_type,
	    std::shared_ptr<google::protobuf::Message> m);
  void send(std::shared_ptr<google::protobuf::Message> m);
  void send(google::protobuf::Message &m);

  /** Get the server's message register.
   * @return message register
   */
  MessageRegister &  message_register()
  { return *message_register_; }

  /** Signal that is invoked when a message has been received.
   * @return signal
   */
  boost::signals2::signal<void (boost::asio::ip::udp::endpoint &, uint16_t, uint16_t,
				std::shared_ptr<google::protobuf::Message>)> &
    signal_received() { return sig_rcvd_; }

  /** Signal that is invoked when receiving a message failed.
   * @return signal
   */
  boost::signals2::signal<void (boost::asio::ip::udp::endpoint &, std::string)> &
    signal_recv_error() { return sig_recv_error_; }

  /** Signal that is invoked when sending a message failed.
   * @return signal
   */
  boost::signals2::signal<void (std::string)> &
    signal_send_error() { return sig_send_error_; }


 private: // methods
  void ctor(const std::string &address, unsigned int send_to_port);
  void determine_local_endpoints();
  void run_asio();
  void start_send();
  void start_recv();
  void handle_resolve(const boost::system::error_code& err,
		      boost::asio::ip::udp::resolver::iterator endpoint_iterator);
  void handle_sent(const boost::system::error_code& error,
		   size_t /*bytes_transferred*/, QueueEntry *entry);
  void handle_recv(const boost::system::error_code& error, size_t bytes_rcvd);

 private: // members
  boost::asio::io_service         io_service_;
  boost::asio::ip::udp::resolver  resolver_;
  boost::asio::ip::udp::socket    socket_;

  std::list<boost::asio::ip::udp::endpoint>  local_endpoints_;

  boost::signals2::signal<void (boost::asio::ip::udp::endpoint &, uint16_t, uint16_t,
				std::shared_ptr<google::protobuf::Message>)> sig_rcvd_;
  boost::signals2::signal<void (boost::asio::ip::udp::endpoint &, std::string)> sig_recv_error_;
  boost::signals2::signal<void (std::string)> sig_send_error_;

  std::string  send_to_address_;

  std::queue<QueueEntry *> outbound_queue_;
  std::mutex               outbound_mutex_;
  bool                     outbound_active_;

  boost::asio::ip::udp::endpoint outbound_endpoint_;
  boost::asio::ip::udp::endpoint in_endpoint_;

  void *         in_data_;
  size_t         in_data_size_;

  bool           filter_self_;

  std::thread asio_thread_;
  MessageRegister *message_register_;
  bool             own_message_register_;
};

} // end namespace protobuf_comm

#endif
