
/***************************************************************************
 * Protoboard plugin template
 * - blackboard manager implementation: Watch interfaces specified in
 *   instantiation and call to protobuf sender thread accordingly.
 *   Also specialize templates for peer handling since that is domain
 *   independent.
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

#include "blackboard_manager.h"

#include "protobuf_to_bb.h"

namespace protoboard {

using namespace fawkes;

AbstractProtobufSender::AbstractProtobufSender(BlackboardManager *bb_mgr) : bb_manager(bb_mgr)
{
}

AbstractProtobufSender::~AbstractProtobufSender()
{
}

BlackboardManager::BlackboardManager(ProtobufThead *msg_handler)
: fawkes::Thread("ProtoboardBlackboardManager", Thread::OPMODE_WAITFORWAKEUP),
  message_handler_(msg_handler),
  bb_receiving_interfaces_(make_receiving_interfaces_map()),
  on_message_waker_(nullptr),
  next_peer_idx_(0)
{
	set_coalesce_wakeups(true);
}

void
BlackboardManager::set_protobuf_sender(AbstractProtobufSender *sender)
{
	pb_sender_.reset(sender);
}

void
BlackboardManager::init()
{
	pb_sender_->init();
	peer_iface_ = blackboard->open_for_writing<ProtobufPeerInterface>("/protoboard/peers");

	on_message_waker_ = new fawkes::BlackBoardOnMessageWaker(blackboard, peer_iface_, this);

	for (pb_conversion_map::value_type &c : bb_receiving_interfaces_)
		c.second->init(blackboard, logger);
}

void
BlackboardManager::finalize()
{
	delete on_message_waker_;
	bb_receiving_interfaces_.clear();
	pb_sender_->finalize();
	blackboard->close(peer_iface_);
}

void
BlackboardManager::loop()
{
	// Handle CreatePeer* messages
	on_interface<ProtobufPeerInterface>{peer_iface_, this}
	  .handle_msg_types<ProtobufPeerInterface::CreatePeerMessage,
	                    ProtobufPeerInterface::CreatePeerLocalMessage,
	                    ProtobufPeerInterface::CreatePeerCryptoMessage,
	                    ProtobufPeerInterface::CreatePeerLocalCryptoMessage>();

	// Handle sending blackboard interfaces
	pb_sender_->process_sending_interfaces();

	// Handle receiving blackboard interfaces
	while (message_handler_->pb_queue_incoming()) {
		ProtobufThead::incoming_message inc = message_handler_->pb_queue_pop();
		pb_conversion_map::iterator     it;

		if ((it = bb_receiving_interfaces_.find(inc.msg->GetTypeName()))
		    == bb_receiving_interfaces_.end()) {
			logger->log_error(name(),
			                  "Received message of unregistered type `%s'",
			                  inc.msg->GetTypeName().c_str());
			continue;
		}
		try {
			it->second->handle(inc.msg);
		} catch (std::exception &e) {
			logger->log_error(name(),
			                  "Exception while handling %s: %s",
			                  inc.msg->GetTypeName().c_str(),
			                  e.what());
		}
	}
}

BlackBoard *
BlackboardManager::get_blackboard()
{
	return blackboard;
}

void
BlackboardManager::add_peer(ProtobufPeerInterface *iface, long peer_id)
{
	if (next_peer_idx_ >= iface->maxlenof_peers()) {
		logger->log_error(name(),
		                  "Maximum number of peers reached. Can't create new peer with index %d!",
		                  next_peer_idx_);
		return;
	}
	iface->set_peers(next_peer_idx_++, peer_id);
	iface->write();
}

/**
 * Local specialization for the CreatePeerMessage that establishes ProtoBuf communication
 * @param iface The ProtobufPeerInterface
 * @param msg The incoming CreatePeerMessage
 */
template <>
void
BlackboardManager::handle_message(ProtobufPeerInterface *                   iface,
                                  ProtobufPeerInterface::CreatePeerMessage *msg)
{
	add_peer(iface, message_handler_->peer_create(msg->address(), msg->port()));
}

/**
 * Local specialization for the CreatePeerLocalMessage that establishes ProtoBuf communication
 * @param iface The ProtobufPeerInterface
 * @param msg The incoming CreatePeerLocalMessage
 */
template <>
void
BlackboardManager::handle_message(ProtobufPeerInterface *                        iface,
                                  ProtobufPeerInterface::CreatePeerLocalMessage *msg)
{
	add_peer(iface,
	         message_handler_->peer_create_local(msg->address(),
	                                             msg->send_to_port(),
	                                             msg->recv_on_port()));
}

/**
 * Local specialization for the CreatePeerCryptoMessage that establishes ProtoBuf communication
 * @param iface The ProtobufPeerInterface
 * @param msg The incoming CreatePeerCryptoMessage
 */
template <>
void
BlackboardManager::handle_message(ProtobufPeerInterface *                         iface,
                                  ProtobufPeerInterface::CreatePeerCryptoMessage *msg)
{
	add_peer(iface,
	         message_handler_->peer_create_crypto(
	           msg->address(), msg->port(), msg->crypto_key(), msg->cipher()));
}

/**
 * Local specialization for the CreatePeerLocalCryptoMessage that establishes ProtoBuf communication
 * @param iface The ProtobufPeerInterface
 * @param msg The incoming CreatePeerLocalCryptoMessage
 */
template <>
void
BlackboardManager::handle_message(ProtobufPeerInterface *                              iface,
                                  ProtobufPeerInterface::CreatePeerLocalCryptoMessage *msg)
{
	add_peer(iface,
	         message_handler_->peer_create_local_crypto(msg->address(),
	                                                    msg->send_to_port(),
	                                                    msg->recv_on_port(),
	                                                    msg->crypto_key(),
	                                                    msg->cipher()));
}

} // namespace protoboard
