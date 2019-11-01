
/***************************************************************************
 * Protoboard plugin template
 * - Implementation of the ProtoBuf thread: protobuf_comm to actually send
 *   messages.
 *
 * Copyright 2019 Victor Mataré
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "protobuf_thread.h"

#include "blackboard_manager.h"

#include <core/threading/mutex_locker.h>
#include <google/protobuf/descriptor.h>
#include <protobuf_comm/client.h>
#include <protobuf_comm/peer.h>
#include <protobuf_comm/server.h>

using namespace google::protobuf;
using namespace protobuf_comm;

namespace protoboard {

ProtobufThead::ProtobufThead()
: Thread("ProtoboardMessageHandler", Thread::OPMODE_CONTINUOUS),
  message_register_(nullptr),
  server_(nullptr),
  next_client_id_(0),
  bb_manager_(nullptr)
{
}

ProtobufThead::~ProtobufThead()
{
	delete message_register_;
	delete server_;
}

void
ProtobufThead::init()
{
	message_register_ = new MessageRegister(proto_dirs());

	if (!bb_manager_)
		throw fawkes::Exception(
		  "BUG: %s's reference to blackboard manager thread hasn't been initialized", name());
}

bool
ProtobufThead::pb_queue_incoming()
{
	fawkes::MutexLocker lock(&msgq_mutex_);
	return !pb_queue_.empty();
}

ProtobufThead::incoming_message
ProtobufThead::pb_queue_pop()
{
	fawkes::MutexLocker lock(&msgq_mutex_);
	incoming_message    msg = pb_queue_.front();
	pb_queue_.pop();
	return msg;
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param send_to_port UDP port to send messages to
 * @param recv_on_port UDP port to receive messages on, 0 to use the same as the @p send_port
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 * @return peer identifier
 */
long int
ProtobufThead::peer_create_local_crypto(const std::string &address,
                                        int                send_to_port,
                                        int                recv_on_port,
                                        const std::string &crypto_key,
                                        const std::string &cipher)
{
	if (recv_on_port <= 0)
		recv_on_port = send_to_port;

	if (send_to_port > 0) {
		protobuf_comm::ProtobufBroadcastPeer *peer = new protobuf_comm::ProtobufBroadcastPeer(
		  address, send_to_port, recv_on_port, message_register_, crypto_key, cipher);

		long int peer_id;
		{
			fawkes::MutexLocker lock(&map_mutex_);
			peer_id         = ++next_client_id_;
			peers_[peer_id] = peer;
		}

		peer->signal_received().connect(
		  boost::bind(&ProtobufThead::handle_peer_msg, this, peer_id, _1, _2, _3, _4));
		peer->signal_recv_error().connect(
		  boost::bind(&ProtobufThead::handle_peer_recv_error, this, peer_id, _1, _2));
		peer->signal_send_error().connect(
		  boost::bind(&ProtobufThead::handle_peer_send_error, this, peer_id, _1));

		return peer_id;
	} else {
		return 0;
	}
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param port UDP port to send and receive messages
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 * @return peer identifier
 */
long int
ProtobufThead::peer_create_crypto(const std::string &address,
                                  int                port,
                                  const std::string &crypto_key,
                                  const std::string &cipher)
{
	return peer_create_local_crypto(address, port, port, crypto_key, cipher);
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param port UDP port to send and receive messages
 * @return peer identifier
 */
long int
ProtobufThead::peer_create(const std::string &address, int port)
{
	return peer_create_local_crypto(address, port, port);
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param send_to_port UDP port to send messages to
 * @param recv_on_port UDP port to receive messages on, 0 to use the same as the @p send_port
 * @return peer identifier
 */
long int
ProtobufThead::peer_create_local(const std::string &address, int send_to_port, int recv_on_port)
{
	return peer_create_local_crypto(address, send_to_port, recv_on_port);
}

/** Disable peer.
 * @param peer_id ID of the peer to destroy
 */
void
ProtobufThead::peer_destroy(long int peer_id)
{
	if (peers_.find(peer_id) != peers_.end()) {
		delete peers_[peer_id];
		peers_.erase(peer_id);
	}
}

/** Setup crypto for peer.
 * @param peer_id ID of the peer to destroy
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 */
void
ProtobufThead::peer_setup_crypto(long int           peer_id,
                                 const std::string &crypto_key,
                                 const std::string &cipher)
{
	if (peers_.find(peer_id) != peers_.end()) {
		peers_[peer_id]->setup_crypto(crypto_key, cipher);
	}
}

void
ProtobufThead::send(long int peer_id, std::shared_ptr<google::protobuf::Message> m)
{
	if (!m) {
		if (logger) {
			logger->log_warn(name(), "Cannot send broadcast: invalid message");
		}
		return;
	}

	fawkes::MutexLocker lock(&map_mutex_);
	if (peers_.find(peer_id) == peers_.end())
		return;

	//logger->log_info(name(), "Broadcasting %s", (*m)->GetTypeName().c_str());
	try {
		peers_[peer_id]->send(m);
	} catch (google::protobuf::FatalException &e) {
		if (logger) {
			logger->log_warn(name(),
			                 "Failed to broadcast message of type %s: %s",
			                 m->GetTypeName().c_str(),
			                 e.what());
		}
	} catch (fawkes::Exception &e) {
		if (logger) {
			logger->log_warn(name(),
			                 "Failed to broadcast message of type %s: %s",
			                 m->GetTypeName().c_str(),
			                 e.what_no_backtrace());
		}
	} catch (std::runtime_error &e) {
		if (logger) {
			logger->log_warn(name(),
			                 "Failed to broadcast message of type %s: %s",
			                 m->GetTypeName().c_str(),
			                 e.what());
		}
	}
}

/** Handle message that came from a peer/robot
 * @param endpoint the endpoint from which the message was received
 * @param component_id component the message was addressed to
 * @param msg_type type of the message
 * @param msg the message
 */
void
ProtobufThead::handle_peer_msg(long int                        peer_id,
                               boost::asio::ip::udp::endpoint &endpoint,
                               uint16_t                        component_id,
                               uint16_t                        msg_type,
                               std::shared_ptr<Message>        msg)
{
	fawkes::MutexLocker lock(&msgq_mutex_);
	pb_queue_.push({peer_id, endpoint, component_id, msg_type, std::move(msg)});
	bb_manager_->wakeup();
}

/** Handle error during peer message processing.
 * @param endpoint endpoint of incoming message
 * @param msg error message
 */
void
ProtobufThead::handle_peer_recv_error(long int                        peer_id,
                                      boost::asio::ip::udp::endpoint &endpoint,
                                      std::string                     msg)
{
	if (logger) {
		logger->log_warn(name(),
		                 "Failed to receive peer message from %s:%u: %s",
		                 endpoint.address().to_string().c_str(),
		                 endpoint.port(),
		                 msg.c_str());
	}
}

/** Handle error during peer message processing.
 * @param msg error message
 */
void
ProtobufThead::handle_peer_send_error(long int peer_id, std::string msg)
{
	if (logger) {
		logger->log_warn(name(), "Failed to send peer message: %s", msg.c_str());
	}
}

} // namespace protoboard
