
/***************************************************************************
 *  client.cpp - Protobuf stream protocol - client
 *
 *  Created: Thu Jan 31 17:38:04 2013
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

#include <protobuf_comm/client.h>

#include <boost/lexical_cast.hpp>

using namespace boost::asio;
using namespace boost::system;

namespace protobuf_comm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ProtobufStreamClient <protobuf_comm/client.h>
 * Stream client for protobuf message transmission.
 * The client opens a TCP connection (IPv4) to a specified server and
 * send and receives messages to the remote.
 * @author Tim Niemueller
 */

/** Constructor. */
ProtobufStreamClient::ProtobufStreamClient()
  : resolver_(io_service_), socket_(io_service_), io_service_work_(io_service_)
{
  message_register_ = new MessageRegister();
  own_message_register_ = true;
  connected_ = false;
  outbound_active_ = false;
  in_data_size_ = 1024;
  frame_header_version_ = PB_FRAME_V2;
  in_frame_header_size_ = sizeof(frame_header_t);
  in_frame_header_ = malloc(in_frame_header_size_);
  in_data_ = malloc(in_data_size_);
  run_asio();
}

/** Constructor.
 * @param proto_path file paths to search for proto files. All message types
 * within these files will automatically be registered and available for dynamic
 * message creation.
 */
ProtobufStreamClient::ProtobufStreamClient(std::vector<std::string> &proto_path)
  : resolver_(io_service_), socket_(io_service_), io_service_work_(io_service_)
{
  message_register_ = new MessageRegister(proto_path);
  own_message_register_ = true;
  connected_ = false;
  outbound_active_ = false;
  in_data_size_ = 1024;
  in_data_ = malloc(in_data_size_);
  frame_header_version_ = PB_FRAME_V2;
  in_frame_header_size_ = sizeof(frame_header_t);
  in_frame_header_ = malloc(in_frame_header_size_);
  run_asio();
}


/** Constructor.
 * @param mr message register to use to (de)serialize messages
 * @param header_version protobuf protocol frame header version to use,
 */
ProtobufStreamClient::ProtobufStreamClient(MessageRegister *mr,
					   frame_header_version_t header_version)
  : resolver_(io_service_), socket_(io_service_), io_service_work_(io_service_),
    message_register_(mr), own_message_register_(false),
    frame_header_version_(header_version)
{
  connected_ = false;
  outbound_active_ = false;
  in_data_size_ = 1024;
  in_data_ = malloc(in_data_size_);
  if (frame_header_version_ == PB_FRAME_V1) {
    in_frame_header_size_ = sizeof(frame_header_v1_t);
  } else {
    in_frame_header_size_ = sizeof(frame_header_t);
  }
  in_frame_header_ = malloc(in_frame_header_size_);
  run_asio();
}


/** Destructor. */
ProtobufStreamClient::~ProtobufStreamClient()
{
  disconnect_nosig();
  io_service_.stop();
  asio_thread_.join();
  free(in_data_);
  free(in_frame_header_);
  if (own_message_register_) {
    delete message_register_;
  }
}


void
ProtobufStreamClient::run_asio()
{
  asio_thread_ = std::thread([this]() { this->io_service_.run(); });
}


/** Asynchronous connect.
 * This triggers connection establishment. The method does not block,
 * i.e. it returns immediately and does not wait for the connection to
 * be established.
 * @param host host to connect to
 * @param port TCP port to connect to
 */
void
ProtobufStreamClient::async_connect(const char *host, unsigned short port)
{
  ip::tcp::resolver::query query(host, boost::lexical_cast<std::string>(port));
  resolver_.async_resolve(query,
			  boost::bind(&ProtobufStreamClient::handle_resolve, this,
				      boost::asio::placeholders::error,
				      boost::asio::placeholders::iterator));

}


void
ProtobufStreamClient::handle_resolve(const boost::system::error_code& err,
				     ip::tcp::resolver::iterator endpoint_iterator)
{
  if (! err) {
    // Attempt a connection to each endpoint in the list until we
    // successfully establish a connection.
#if BOOST_ASIO_VERSION > 100409
    boost::asio::async_connect(socket_, endpoint_iterator,
#else
    socket_.async_connect(*endpoint_iterator,
#endif
			       boost::bind(&ProtobufStreamClient::handle_connect, this,
					   boost::asio::placeholders::error));
  } else {
    disconnect_nosig();
    sig_disconnected_(err);
  }
}

void
ProtobufStreamClient::handle_connect(const boost::system::error_code &err)
{
  if (! err) {
    connected_ = true;
    start_recv();
    sig_connected_();
  } else {
    disconnect_nosig();
    sig_disconnected_(err);
  }
}

void
ProtobufStreamClient::disconnect_nosig()
{
  boost::system::error_code err;
  if (socket_.is_open()) {
    socket_.shutdown(ip::tcp::socket::shutdown_both, err);
    socket_.close();
  }
  connected_ = false;
}


/** Disconnect from remote host. */
void
ProtobufStreamClient::disconnect()
{
  disconnect_nosig();
  sig_disconnected_(boost::system::error_code());
}


void
ProtobufStreamClient::start_recv()
{
  boost::asio::async_read(socket_,
			  boost::asio::buffer(in_frame_header_, in_frame_header_size_),
			  boost::bind(&ProtobufStreamClient::handle_read_header,
				      this, boost::asio::placeholders::error));
}

void
ProtobufStreamClient::handle_read_header(const boost::system::error_code& error)
{
  if (! error) {
    size_t to_read;
    if (frame_header_version_ == PB_FRAME_V1) {
      frame_header_v1_t *frame_header = (frame_header_v1_t *)in_frame_header_;
      to_read = ntohl(frame_header->payload_size);
    } else {
      frame_header_t *frame_header = (frame_header_t *)in_frame_header_;
      to_read = ntohl(frame_header->payload_size);
    }
    if (to_read > in_data_size_) {
      void *new_data = realloc(in_data_, to_read);
      if (new_data) {
	in_data_size_ = to_read;
	in_data_ = new_data;
      } else {
	disconnect_nosig();
	sig_disconnected_(errc::make_error_code(errc::not_enough_memory));
      }
    }
    // setup new read
    boost::asio::async_read(socket_,
			    boost::asio::buffer(in_data_, to_read),
			    boost::bind(&ProtobufStreamClient::handle_read_message,
					this, boost::asio::placeholders::error));
  } else {
    disconnect_nosig();
    sig_disconnected_(error);
  }
}

void
ProtobufStreamClient::handle_read_message(const boost::system::error_code& error)
{
  if (! error) {
    frame_header_t frame_header;
    message_header_t message_header;
    void *data;

    if (frame_header_version_ == PB_FRAME_V1) {
      frame_header_v1_t *frame_header_v1 = (frame_header_v1_t *)in_frame_header_;
      frame_header.header_version = PB_FRAME_V1;
      frame_header.cipher         = PB_ENCRYPTION_NONE;
      frame_header.payload_size   = htonl(ntohl(frame_header_v1->payload_size) + sizeof(message_header_t));
      message_header.component_id = frame_header_v1->component_id;
      message_header.msg_type     = frame_header_v1->msg_type;
      data = in_data_;
    } else {
      memcpy(&frame_header, in_frame_header_, sizeof(frame_header_t));

      message_header_t *msg_header = static_cast<message_header_t *>(in_data_);
      message_header.component_id = msg_header->component_id;
      message_header.msg_type     = msg_header->msg_type;

      data = (char *)in_data_ + sizeof(message_header);
    }

    uint16_t comp_id   = ntohs(message_header.component_id);
    uint16_t msg_type  = ntohs(message_header.msg_type);
    try {
      std::shared_ptr<google::protobuf::Message> m =
	message_register_->deserialize(frame_header, message_header, data);

      sig_rcvd_(comp_id, msg_type, m);
    } catch (std::runtime_error &e) {
      sig_recv_failed_(comp_id, msg_type, e.what());
    }

    start_recv();
  } else {
    disconnect_nosig();
    sig_disconnected_(error);
  }
}

void
ProtobufStreamClient::handle_write(const boost::system::error_code& error,
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
			       boost::bind(&ProtobufStreamClient::handle_write, this,
					   boost::asio::placeholders::error,
					   boost::asio::placeholders::bytes_transferred,
					   entry));
    } else {
      outbound_active_ = false;
    }
  } else {
    disconnect_nosig();
    sig_disconnected_(error);
  }
}


/** Send a message to the server.
 * @param component_id ID of the component to address
 * @param msg_type numeric message type
 * @param m message to send
 */
void
ProtobufStreamClient::send(uint16_t component_id, uint16_t msg_type,
			   google::protobuf::Message &m)
{
  if (!connected_) {
    throw std::runtime_error("Cannot send while not connected");
  }

  QueueEntry *entry = new QueueEntry();
  message_register_->serialize(component_id, msg_type, m,
			       entry->frame_header, entry->message_header,
			       entry->serialized_message);

  if (frame_header_version_ == PB_FRAME_V1) {
    entry->frame_header_v1.component_id = entry->message_header.component_id;
    entry->frame_header_v1.msg_type     = entry->message_header.msg_type;
    entry->frame_header_v1.payload_size =
      htonl(ntohl(entry->frame_header.payload_size) - sizeof(message_header_t));

    entry->buffers[0] = boost::asio::buffer(&entry->frame_header_v1, sizeof(frame_header_v1_t));
    entry->buffers[1] = boost::asio::const_buffer();
  } else {
    entry->buffers[0] = boost::asio::buffer(&entry->frame_header, sizeof(frame_header_t));
    entry->buffers[1] = boost::asio::buffer(&entry->message_header, sizeof(message_header_t));
  }
  entry->buffers[2] = boost::asio::buffer(entry->serialized_message);
 
  std::lock_guard<std::mutex> lock(outbound_mutex_);
  if (outbound_active_) {
    outbound_queue_.push(entry);
  } else {
    outbound_active_ = true;
    boost::asio::async_write(socket_, entry->buffers,
			     boost::bind(&ProtobufStreamClient::handle_write, this,
					 boost::asio::placeholders::error,
					 boost::asio::placeholders::bytes_transferred,
					 entry));
  }
}


/** Send a message to the server.
 * @param m message to send, the message must be of a type with a suitable CompType
 * enum indicating component ID and message type.
 */
void
ProtobufStreamClient::send(google::protobuf::Message &m)
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

  send(comp_id, msg_type, m);
}


/** Send a message to the server.
 * @param m message to send, the message must be of a type with a suitable CompType
 * enum indicating component ID and message type.
 */
void
ProtobufStreamClient::send(std::shared_ptr<google::protobuf::Message> m)
{
  send(*m);
}

} // end namespace protobuf_comm
