
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

class BufferEncryptor;
class BufferDecryptor;

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
			unsigned short recv_on_port, MessageRegister *mr,
			frame_header_version_t header_version = PB_FRAME_V2);
  ProtobufBroadcastPeer(const std::string address, unsigned short port,
			const std::string crypto_key, const std::string cipher = "aes-128-ecb");
  ProtobufBroadcastPeer(const std::string address, unsigned short port, MessageRegister *mr,
			const std::string crypto_key, const std::string cipher = "aes-128-ecb");
  ProtobufBroadcastPeer(const std::string address, unsigned short send_to_port,
			unsigned short recv_on_port,
			const std::string crypto_key, const std::string cipher = "aes-128-ecb");
  ProtobufBroadcastPeer(const std::string address, unsigned short send_to_port,
			unsigned short recv_on_port, MessageRegister *mr,
			const std::string crypto_key, const std::string cipher = "aes-128-ecb");
  ~ProtobufBroadcastPeer();

  void set_filter_self(bool filter);

  void send(uint16_t component_id, uint16_t msg_type,
	    google::protobuf::Message &m);
  void send(uint16_t component_id, uint16_t msg_type,
	    std::shared_ptr<google::protobuf::Message> m);
  void send(std::shared_ptr<google::protobuf::Message> m);
  void send(google::protobuf::Message &m);

  void send_raw(const frame_header_t &frame_header, const void *data, size_t data_size);

  void setup_crypto(const std::string &key, const std::string &cipher);

  /** Get the server's message register.
   * @return message register
   */
  MessageRegister &  message_register()
  { return *message_register_; }

  /** Boost signal for a received message. */
  typedef
    boost::signals2::signal<void (boost::asio::ip::udp::endpoint &, uint16_t, uint16_t,
				  std::shared_ptr<google::protobuf::Message>)>
    signal_received_type;

  /** Boost signal for a received raw message. */
  typedef
    boost::signals2::signal<void (boost::asio::ip::udp::endpoint &, frame_header_t &,
				  void *, size_t)>
    signal_received_raw_type;

  /** Boost signal for an error during receiving a message. */
  typedef
    boost::signals2::signal<void (boost::asio::ip::udp::endpoint &, std::string)>
    signal_recv_error_type;

  /** Boost signal for an error during sending a message. */
  typedef
    boost::signals2::signal<void (std::string)>
    signal_send_error_type;

  /** Signal that is invoked when a message has been received.
   * @return signal
   */
  signal_received_type &  signal_received()
  { return sig_rcvd_; }

  /** Signal that is invoked when a message has been received.
   * This allows access to the raw packet data. This allows, for example,
   * to write an ecryption agnostic repeater.
   * @return signal
   */
  signal_received_raw_type &  signal_received_raw()
  { return sig_rcvd_raw_; }

  /** Signal that is invoked when receiving a message failed.
   * @return signal
   */
   signal_recv_error_type &  signal_recv_error()
   { return sig_recv_error_; }

  /** Signal that is invoked when sending a message failed.
   * @return signal
   */
  signal_send_error_type &  signal_send_error()
  { return sig_send_error_; }


 private: // methods
  void ctor(const std::string &address, unsigned int send_to_port,
	    const std::string crypto_key = "", const std::string cipher = "aes-128-ecb",
	    frame_header_version_t = PB_FRAME_V2);
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

  signal_received_type     sig_rcvd_;
  signal_received_raw_type sig_rcvd_raw_;
  signal_recv_error_type   sig_recv_error_;
  signal_send_error_type   sig_send_error_;

  std::string  send_to_address_;

  std::queue<QueueEntry *> outbound_queue_;
  std::mutex               outbound_mutex_;
  bool                     outbound_active_;

  boost::asio::ip::udp::endpoint outbound_endpoint_;
  boost::asio::ip::udp::endpoint in_endpoint_;

  void *         in_data_;
  void *         enc_in_data_;
  size_t         in_data_size_;
  size_t         enc_in_data_size_;

  bool           filter_self_;

  std::thread asio_thread_;
  MessageRegister *message_register_;
  bool             own_message_register_;

  frame_header_version_t frame_header_version_;

  bool             crypto_;
  bool             crypto_buf_;
  BufferEncryptor *crypto_enc_;
  BufferDecryptor *crypto_dec_;
};

} // end namespace protobuf_comm

#endif
