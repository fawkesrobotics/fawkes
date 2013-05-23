
/***************************************************************************
 *  server.h - Protobuf stream protocol - server
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

#ifndef __PROTOBUF_COMM_SERVER_H_
#define __PROTOBUF_COMM_SERVER_H_

#include <protobuf_comm/frame_header.h>
#include <protobuf_comm/message_register.h>
#include <protobuf_comm/queue_entry.h>

#include <boost/asio.hpp>
#include <boost/signals2.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <google/protobuf/message.h>

#ifndef _GLIBCXX_USE_SCHED_YIELD
#  define _GLIBCXX_USE_SCHED_YIELD
#endif
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>

namespace protobuf_comm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ProtobufStreamServer
{
 public:
  /** ID to identify connected clients. */
  typedef unsigned int ClientID;

  ProtobufStreamServer(unsigned short port);
  ProtobufStreamServer(unsigned short port, std::vector<std::string> &proto_path);
  ProtobufStreamServer(unsigned short port, MessageRegister *mr);
  ~ProtobufStreamServer();

  void send(ClientID client, uint16_t component_id, uint16_t msg_type,
	    google::protobuf::Message &m);
  void send(ClientID client, uint16_t component_id, uint16_t msg_type,
	    std::shared_ptr<google::protobuf::Message> m);
  void send(ClientID client, std::shared_ptr<google::protobuf::Message> m);
  void send(ClientID client, google::protobuf::Message &m);

  void send_to_all(uint16_t component_id, uint16_t msg_type,
		   google::protobuf::Message &m);
  void send_to_all(uint16_t component_id, uint16_t msg_type,
		   std::shared_ptr<google::protobuf::Message> m);
  void send_to_all(std::shared_ptr<google::protobuf::Message> m);
  void send_to_all(google::protobuf::Message &m);

  void disconnect(ClientID client);

  /** Get the server's message register.
   * @return message register
   */
  MessageRegister &  message_register()
  { return *message_register_; }

  /** Signal that is invoked when a message has been received.
   * @return signal
   */
  boost::signals2::signal<void (ClientID, uint16_t, uint16_t,
				std::shared_ptr<google::protobuf::Message>)> &
    signal_received() { return sig_rcvd_; }

  /** Signal that is invoked when receiving a message failed.
   * @return signal
   */
  boost::signals2::signal<void (ClientID, uint16_t, uint16_t, std::string)> &
    signal_receive_failed() { return sig_recv_failed_; }

  /** Signal that is invoked when a new client has connected.
   * @return signal
   */
  boost::signals2::signal<void (ClientID, boost::asio::ip::tcp::endpoint &)> &
    signal_connected() { return sig_connected_; }

  /** Signal that is invoked when a new client has disconnected.
   * @return signal
   */
  boost::signals2::signal<void (ClientID, const boost::system::error_code &)> &
    signal_disconnected() { return sig_disconnected_; }

 private:
  class Session : public boost::enable_shared_from_this<Session>
  {
   public:
    /** Shortcut for shared pointer of session. */
    typedef boost::shared_ptr<Session> Ptr;

    Session(ClientID id, ProtobufStreamServer *parent,
	    boost::asio::io_service &io_service);
    ~Session();

    /** Get underlying socket.
     * @return socket */
    boost::asio::ip::tcp::socket & socket() { return socket_; }

    /** Get client ID.
     * @return client ID */
    ClientID id() const { return id_; }
    /** Get client's endpoint.
     * @return remote client's endpoint */
    boost::asio::ip::tcp::endpoint & remote_endpoint()
    { return remote_endpoint_; }

    void start_session();
    void start_read();
    void send(uint16_t component_id, uint16_t msg_type,
	      google::protobuf::Message &m);
    void disconnect();

   private:
    void handle_read_message(const boost::system::error_code& error);
    void handle_read_header(const boost::system::error_code& error);
    void handle_write(const boost::system::error_code& error,
		      size_t /*bytes_transferred*/, QueueEntry *entry);

   private:
    ClientID id_;
    ProtobufStreamServer *parent_;
    boost::asio::ip::tcp::socket socket_;
    boost::asio::ip::tcp::endpoint remote_endpoint_;

    frame_header_t in_frame_header_;
    size_t         in_data_size_;
    void *         in_data_;

    std::queue<QueueEntry *> outbound_queue_;
    std::mutex               outbound_mutex_;
    bool                     outbound_active_;
  };

 private: // methods
  void run_asio();
  void start_accept();
  void handle_accept(Session::Ptr new_session, const boost::system::error_code& error);

  void disconnected(boost::shared_ptr<Session> session,
		    const boost::system::error_code &error);

 private: // members
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::signals2::signal<void (ClientID, uint16_t, uint16_t,
				std::shared_ptr<google::protobuf::Message>)> sig_rcvd_;
  boost::signals2::signal<void (ClientID, uint16_t, uint16_t, std::string)> sig_recv_failed_;
  boost::signals2::signal<void (ClientID, boost::asio::ip::tcp::endpoint &)> sig_connected_;
  boost::signals2::signal<void (ClientID, const boost::system::error_code &)>
    sig_disconnected_;

  std::thread asio_thread_;

  std::map<ClientID, boost::shared_ptr<Session>> sessions_;

  std::atomic<ClientID> next_cid_;

  MessageRegister *message_register_;
  bool             own_message_register_;
};

} // end namespace protobuf_comm

#endif
