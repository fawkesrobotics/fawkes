
/***************************************************************************
 *  server.cpp - Protobuf stream protocol - server
 *
 *  Created: Thu Jan 31 14:57:16 2013
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

#include <protobuf_comm/server.h>

#include <cstdlib>

using namespace boost::asio;
using namespace boost::system;

namespace protobuf_comm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ProtobufStreamServer::Session <protobuf_comm/server.h>
 * Internal class representing a client session.
 * This class represents a connection to a particular client. It handles
 * connection management, reading from, and writing to the client.
 * @author Tim Niemueller 
 */

/** Constructor.
 * @param id ID of the client, used to address messages from within your
 * application.
 * @param parent Parent stream server notified about events.
 * @param io_service ASIO I/O service to use for communication
 */
ProtobufStreamServer::Session::Session(ClientID id, ProtobufStreamServer *parent,
				       boost::asio::io_service& io_service)
  : id_(id), parent_(parent), socket_(io_service)
{
  in_data_size_ = 1024;
  in_data_ = malloc(in_data_size_);
  outbound_active_ = false;
}

/** Destructor. */
ProtobufStreamServer::Session::~Session()
{
  boost::system::error_code err;
  if (socket_.is_open()) {
    socket_.shutdown(ip::tcp::socket::shutdown_both, err);
    socket_.close();
  }
  free(in_data_);
}

/** Do processing required to start a session.
 */
void
ProtobufStreamServer::Session::start_session()
{
  remote_endpoint_ = socket_.remote_endpoint();
}

/** Start reading a message on this session.
 * This sets up a read handler to read incoming messages. It also notifies
 * the parent server of the initiated connection.
 */
void
ProtobufStreamServer::Session::start_read()
{
  boost::asio::async_read(socket_,
			  boost::asio::buffer(&in_frame_header_, sizeof(frame_header_t)),
			  boost::bind(&ProtobufStreamServer::Session::handle_read_header,
				      shared_from_this(), boost::asio::placeholders::error));
}


 /** Send a message.
 * @param component_id ID of the component to address
 * @param msg_type numeric message type
 * @param m Message to send
 */
void
ProtobufStreamServer::Session::send(uint16_t component_id, uint16_t msg_type,
				    google::protobuf::Message &m)
{
  QueueEntry *entry = new QueueEntry();
  parent_->message_register().serialize(component_id, msg_type, m,
					entry->frame_header, entry->message_header,
					entry->serialized_message);

  entry->buffers[0] = boost::asio::buffer(&entry->frame_header, sizeof(frame_header_t));
  entry->buffers[1] = boost::asio::buffer(&entry->message_header, sizeof(message_header_t));
  entry->buffers[2] = boost::asio::buffer(entry->serialized_message);
 
  std::lock_guard<std::mutex> lock(outbound_mutex_);
  if (outbound_active_) {
    outbound_queue_.push(entry);
  } else {
    outbound_active_ = true;
    boost::asio::async_write(socket_, entry->buffers,
			     boost::bind(&ProtobufStreamServer::Session::handle_write,
					 shared_from_this(),
					 boost::asio::placeholders::error,
					 boost::asio::placeholders::bytes_transferred,
					 entry));
  }
}


/** Disconnect from client. */
void
ProtobufStreamServer::Session::disconnect()
{
  boost::system::error_code err;
  if (socket_.is_open()) {
    socket_.shutdown(ip::tcp::socket::shutdown_both, err);
    socket_.close();
  }
}


/** Write completion handler. */
void
ProtobufStreamServer::Session::handle_write(const boost::system::error_code& error,
					    size_t /*bytes_transferred*/,
					    QueueEntry *entry)
{
  delete entry;

  if (! error) {
    std::lock_guard<std::mutex> lock(outbound_mutex_);
    if (! outbound_queue_.empty()) {
      QueueEntry *entry = outbound_queue_.front();
      outbound_queue_.pop();
      boost::asio::async_write(socket_, entry->buffers,
			       boost::bind(&ProtobufStreamServer::Session::handle_write,
					   shared_from_this(),
					   boost::asio::placeholders::error,
					   boost::asio::placeholders::bytes_transferred,
					   entry));
    } else {
      outbound_active_ = false;
    }
  } else {
    parent_->disconnected(shared_from_this(), error);
  }
}


/** Incoming data handler for header.
 * This method is called if an error occurs while waiting for data (e.g. if
 * the remote peer closes the connection), or if new data is available. This
 * callback expectes header information to be received.
 * @param error error code
 */
void
ProtobufStreamServer::Session::handle_read_header(const boost::system::error_code& error)
{
  if (! error) {
    size_t to_read = ntohl(in_frame_header_.payload_size);
    if (to_read > in_data_size_) {
      void *new_data = realloc(in_data_, to_read);
      if (new_data) {
	in_data_size_ = to_read;
	in_data_ = new_data;
      } else {
	parent_->disconnected(shared_from_this(),
			      errc::make_error_code(errc::not_enough_memory));
      }
    }
    // setup new read
    boost::asio::async_read(socket_,
			    boost::asio::buffer(in_data_, to_read),
			    boost::bind(&ProtobufStreamServer::Session::handle_read_message,
					shared_from_this(), boost::asio::placeholders::error));
  } else {
    parent_->disconnected(shared_from_this(), error);
  }
}


/** Incoming data handler for message content.
 * This method is called if an error occurs while waiting for data (e.g. if
 * the remote peer closes the connection), or if new data is available. This
 * callback expectes message to be received that conforms to a previously
 * received header.
 * @param error error code
 */
void
ProtobufStreamServer::Session::handle_read_message(const boost::system::error_code& error)
{
  if (! error) {
    message_header_t *message_header = static_cast<message_header_t *>(in_data_);

    uint16_t comp_id   = ntohs(message_header->component_id);
    uint16_t msg_type  = ntohs(message_header->msg_type);
    try {
      std::shared_ptr<google::protobuf::Message> m =
	parent_->message_register().deserialize(in_frame_header_, *message_header,
						(char *)in_data_ + sizeof(message_header_t));
      parent_->sig_rcvd_(id_, comp_id, msg_type, m);
    } catch (std::runtime_error &e) {
      // ignored, most likely unknown message tpye
      parent_->sig_recv_failed_(id_, comp_id, msg_type, e.what());
    }
    start_read();
  } else {
    parent_->disconnected(shared_from_this(), error);
  }
}


/** @class ProtobufStreamServer <protobuf_comm/server.h>
 * Stream server for protobuf message transmission.
 * The server opens a TCP socket (IPv4) and waits for incoming connections.
 * Each incoming connection is given a unique client ID. Signals are
 * provided that can be used to react to connections and incoming data.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param port port to listen on
 */
ProtobufStreamServer::ProtobufStreamServer(unsigned short port)
  : io_service_(),
    acceptor_(io_service_, ip::tcp::endpoint(ip::tcp::v6(), port))
{
  message_register_ = new MessageRegister();
  own_message_register_ = true;
  next_cid_ = 1;

  acceptor_.set_option(socket_base::reuse_address(true));

  start_accept();
  asio_thread_ = std::thread(&ProtobufStreamServer::run_asio, this);
}


/** Constructor.
 * @param port port to listen on
 * @param proto_path file paths to search for proto files. All message types
 * within these files will automatically be registered and available for dynamic
 * message creation.
 */
ProtobufStreamServer::ProtobufStreamServer(unsigned short port,
					   std::vector<std::string> &proto_path)
  : io_service_(),
    acceptor_(io_service_, ip::tcp::endpoint(ip::tcp::v6(), port))
{
  message_register_ = new MessageRegister(proto_path);
  own_message_register_ = true;
  next_cid_ = 1;

  acceptor_.set_option(socket_base::reuse_address(true));

  start_accept();
  asio_thread_ = std::thread(&ProtobufStreamServer::run_asio, this);
}

/** Constructor.
 * @param port port to listen on
 * @param mr message register to use to (de)serialize messages
 */
ProtobufStreamServer::ProtobufStreamServer(unsigned short port,
					   MessageRegister *mr)
  : io_service_(),
    acceptor_(io_service_, ip::tcp::endpoint(ip::tcp::v6(), port)),
    message_register_(mr), own_message_register_(false)
{
  next_cid_ = 1;

  acceptor_.set_option(socket_base::reuse_address(true));

  start_accept();
  asio_thread_ = std::thread(&ProtobufStreamServer::run_asio, this);
}


/** Destructor. */
ProtobufStreamServer::~ProtobufStreamServer()
{
  io_service_.stop();
  asio_thread_.join();
  if (own_message_register_) {
    delete message_register_;
  }
}


/** Send a message to the given client.
 * @param client ID of the client to addresss
 * @param component_id ID of the component to address
 * @param msg_type numeric message type
 * @param m message to send
 */
void
ProtobufStreamServer::send(ClientID client, uint16_t component_id, uint16_t msg_type,
			   google::protobuf::Message &m)
{
  if (sessions_.find(client) == sessions_.end()) {
    throw std::runtime_error("Client does not exist");
  }

  sessions_[client]->send(component_id, msg_type, m);
}


/** Send a message.
 * @param client ID of the client to addresss
 * @param component_id ID of the component to address
 * @param msg_type numeric message type
 * @param m Message to send
 */
void
ProtobufStreamServer::send(ClientID client, uint16_t component_id, uint16_t msg_type,
			   std::shared_ptr<google::protobuf::Message> m)
{
  send(client, component_id, msg_type, *m);
}

/** Send a message.
 * @param client ID of the client to addresss
 * @param m Message to send, the message must have an CompType enum type to
 * specify component ID and message type.
 */
void
ProtobufStreamServer::send(ClientID client, google::protobuf::Message &m)
{
  const google::protobuf::Descriptor *desc = m.GetDescriptor();
  const google::protobuf::EnumDescriptor *enumdesc = desc->FindEnumTypeByName("CompType");
  if (! enumdesc) {
    throw std::logic_error("Message does not have CompType enum");
  }
  const google::protobuf::EnumValueDescriptor *compdesc =
    enumdesc->FindValueByName("COMP_ID");
  const google::protobuf::EnumValueDescriptor *msgtdesc =
    enumdesc->FindValueByName("MSG_TYPE");
  if (! compdesc || ! msgtdesc) {
    throw std::logic_error("Message CompType enum hs no COMP_ID or MSG_TYPE value");
  }
  int comp_id = compdesc->number();
  int msg_type = msgtdesc->number();
  if (comp_id < 0 || comp_id > std::numeric_limits<uint16_t>::max()) {
    throw std::logic_error("Message has invalid COMP_ID");
  }
  if (msg_type < 0 || msg_type > std::numeric_limits<uint16_t>::max()) {
    throw std::logic_error("Message has invalid MSG_TYPE");
  }

  send(client, comp_id, msg_type, m);
}

/** Send a message.
 * @param client ID of the client to addresss
 * @param m Message to send, the message must have an CompType enum type to
 * specify component ID and message type.
 */
void
ProtobufStreamServer::send(ClientID client, std::shared_ptr<google::protobuf::Message> m)
{
  send(client, *m);
}

/** Send a message to all clients.
 * @param component_id ID of the component to address
 * @param msg_type numeric message type
 * @param m message to send
 */
void
ProtobufStreamServer::send_to_all(uint16_t component_id, uint16_t msg_type,
				  google::protobuf::Message &m)
{
  std::map<ClientID, boost::shared_ptr<Session>>::iterator s;
  for (s = sessions_.begin(); s != sessions_.end(); ++s) {
    send(s->first, component_id, msg_type, m);
  }
}

/** Send a message to all clients.
 * @param component_id ID of the component to address
 * @param msg_type numeric message type
 * @param m message to send
 */
void
ProtobufStreamServer::send_to_all(uint16_t component_id, uint16_t msg_type,
				  std::shared_ptr<google::protobuf::Message> m)
{
  std::map<ClientID, boost::shared_ptr<Session>>::iterator s;
  for (s = sessions_.begin(); s != sessions_.end(); ++s) {
    send(s->first, component_id, msg_type, m);
  }
}

/** Send a message to all clients.
 * @param m message to send
 */
void
ProtobufStreamServer::send_to_all(std::shared_ptr<google::protobuf::Message> m)
{
  std::map<ClientID, boost::shared_ptr<Session>>::iterator s;
  for (s = sessions_.begin(); s != sessions_.end(); ++s) {
    send(s->first, m);
  }
}

/** Send a message to all clients.
 * @param m message to send
 */
void
ProtobufStreamServer::send_to_all(google::protobuf::Message &m)
{
  std::map<ClientID, boost::shared_ptr<Session>>::iterator s;
  for (s = sessions_.begin(); s != sessions_.end(); ++s) {
    send(s->first, m);
  }
}


/** Disconnect specific client.
 * @param client client ID to disconnect from
 */
void
ProtobufStreamServer::disconnect(ClientID client)
{
  if (sessions_.find(client) != sessions_.end()) {
    boost::shared_ptr<Session> session = sessions_[client];
    session->disconnect();
  }
}

/** Start accepting connections. */
void
ProtobufStreamServer::start_accept()
{
  Session::Ptr new_session(new Session(next_cid_++, this, io_service_));
  acceptor_.async_accept(new_session->socket(),
			 boost::bind(&ProtobufStreamServer::handle_accept, this,
				     new_session, boost::asio::placeholders::error));
}

void
ProtobufStreamServer::disconnected(boost::shared_ptr<Session> session,
				   const boost::system::error_code &error)
{
  sessions_.erase(session->id());
  sig_disconnected_(session->id(), error);
}

void
ProtobufStreamServer::handle_accept(Session::Ptr new_session,
				    const boost::system::error_code& error)
{
  if (!error) {
    new_session->start_session();
    sessions_[new_session->id()] = new_session;
    sig_connected_(new_session->id(), new_session->remote_endpoint());
    new_session->start_read();
  }

  start_accept();
}


void
ProtobufStreamServer::run_asio()
{
#if BOOST_ASIO_VERSION > 100409
  while (! io_service_.stopped()) {
#endif
    usleep(0);
    io_service_.reset();
    io_service_.run();
#if BOOST_ASIO_VERSION > 100409
  }
#endif
}

} // end namespace protobuf_comm
