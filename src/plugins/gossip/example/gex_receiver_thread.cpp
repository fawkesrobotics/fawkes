
/***************************************************************************
 *  gex_receiver_thread.cpp - Gossip Example Plugin - Receiver
 *
 *  Created: Thu Mar 06 10:40:11 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include "gex_receiver_thread.h"

#include "TestMessage.pb.h"

using namespace fawkes;

/** @class GossipExampleReceiverThread "clips-protobuf-thread.h"
 * Gossip Example Plugin Thread - Receiver.
 * @author Tim Niemueller
 */

/** Constructor. */
GossipExampleReceiverThread::GossipExampleReceiverThread()
  : Thread("GossipExampleReceiverThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
    GossipAspect("example")
{
}


/** Destructor. */
GossipExampleReceiverThread::~GossipExampleReceiverThread()
{
}


void
GossipExampleReceiverThread::init()
{
  try {
    gossip_group->message_register().add_message_type<gossip_example::TestMessage>();
  } catch (std::runtime_error &e) {} // ignore, probably already added

  sig_rcvd_conn_ =
    gossip_group->signal_received()
    .connect(boost::bind(&GossipExampleReceiverThread::handle_peer_msg, this, _1, _2, _3, _4));

  sig_recv_error_conn_ =
    gossip_group->signal_recv_error()
    .connect(boost::bind(&GossipExampleReceiverThread::handle_peer_recv_error, this, _1, _2));

  sig_send_error_conn_ =
    gossip_group->signal_send_error()
    .connect(boost::bind(&GossipExampleReceiverThread::handle_peer_send_error, this, _1));
}


void
GossipExampleReceiverThread::finalize()
{
  sig_rcvd_conn_.disconnect();
  sig_recv_error_conn_.disconnect();
  sig_send_error_conn_.disconnect();
}


void
GossipExampleReceiverThread::loop()
{
}

void
GossipExampleReceiverThread::handle_peer_msg(boost::asio::ip::udp::endpoint &endpoint,
					     uint16_t component_id, uint16_t msg_type,
					     std::shared_ptr<google::protobuf::Message> msg)
{
  if (component_id == gossip_example::TestMessage::COMP_ID &&
      msg_type == gossip_example::TestMessage::MSG_TYPE)
  {
    std::shared_ptr<gossip_example::TestMessage> tm =
      std::dynamic_pointer_cast<gossip_example::TestMessage>(msg);
    if (tm) {
      logger->log_info(name(), "Received message with counter %u", tm->counter());
    } else {
      logger->log_warn(name(), "Message with proper component_id and msg_type, but no conversion. "
		       " Wrong component ID/message type to C++ type mapping?");
    }
  } else {
    logger->log_warn(name(), "Unknown message received: %u:%u", component_id, msg_type);
  }
}

/** Handle error during peer message processing.
 * @param endpoint endpoint of incoming message
 * @param msg error message
 */
void
GossipExampleReceiverThread::handle_peer_recv_error(boost::asio::ip::udp::endpoint &endpoint,
						    std::string msg)
{
  logger->log_warn(name(), "Failed to receive peer message from %s:%u: %s",
		   endpoint.address().to_string().c_str(), endpoint.port(), msg.c_str());
}

/** Handle error during peer message processing.
 * @param msg error message
 */
void
GossipExampleReceiverThread::handle_peer_send_error(std::string msg)
{
  logger->log_warn(name(), "Failed to send peer message: %s", msg.c_str());
}

