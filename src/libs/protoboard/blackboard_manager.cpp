
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
	blackboard->close(peer_iface_);
}

void
BlackboardManager::loop()
{
	// Handle CreatePeer* messages
	bool did_something = on_interface<ProtobufPeerInterface>{peer_iface_, this}
	                       .handle_msg_types<ProtobufPeerInterface::CreatePeerMessage,
	                                         ProtobufPeerInterface::CreatePeerLocalMessage,
	                                         ProtobufPeerInterface::CreatePeerCryptoMessage,
	                                         ProtobufPeerInterface::CreatePeerLocalCryptoMessage>();

	// Handle sending blackboard interfaces
	did_something |= pb_sender_->process_sending_interfaces();

	// Handle receiving blackboard interfaces
	while (message_handler_->pb_queue_incoming()) {
		ProtobufThead::incoming_message inc = message_handler_->pb_queue_pop();
		pb_conversion_map::iterator     it;

		if ((it = bb_receiving_interfaces_.find(inc.msg->GetTypeName()))
		    == bb_receiving_interfaces_.end())
			logger->log_error(name(),
			                  "Received message of unregistered type `%s'",
			                  inc.msg->GetTypeName().c_str());
		else
			try {
				it->second->handle(inc.msg);
			} catch (std::exception &e) {
				logger->log_error(name(),
				                  "Exception while handling %s: %s",
				                  inc.msg->GetTypeName().c_str(),
				                  e.what());
			}

		did_something = true;
	}

	if (!did_something)
		// Thread woke up, but nothing was handled
		logger->log_warn(name(), "Spurious wakeup. WTF?");
}

BlackBoard *
BlackboardManager::get_blackboard()
{
	return blackboard;
}

void
BlackboardManager::add_peer(ProtobufPeerInterface *iface, long peer_id)
{
	// TODO: Properly handle overflow.
	iface->set_peers(next_peer_idx_++ % iface->maxlenof_peers(), peer_id);
	iface->write();
}

template <>
void
BlackboardManager::handle_message(ProtobufPeerInterface *                   iface,
                                  ProtobufPeerInterface::CreatePeerMessage *msg)
{
	add_peer(iface, message_handler_->peer_create(msg->address(), msg->port()));
}

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

template <>
void
BlackboardManager::handle_message(ProtobufPeerInterface *                         iface,
                                  ProtobufPeerInterface::CreatePeerCryptoMessage *msg)
{
	add_peer(iface,
	         message_handler_->peer_create_crypto(
	           msg->address(), msg->port(), msg->crypto_key(), msg->cipher()));
}

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
