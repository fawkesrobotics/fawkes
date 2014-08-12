
/***************************************************************************
 *  gossip_group.cpp - Robot Group Communication - Gossip Group
 *
 *  Created: Tue Mar 04 11:00:11 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <plugins/gossip/gossip/gossip_group.h>

#include <netcomm/service_discovery/service.h>
#include <netcomm/service_discovery/service_publisher.h>

#include <protobuf_comm/peer.h>

#define GOSSIP_MDNSSD_SERVICE_NAME "_gossip._udp"


namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class GossipGroup <plugins/gossip/gossip/gossip_group.h>
 * Gossip group communication handler.
 * The group communication handler cares about joining groups and
 * sending and receiving data.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param group_name name of the group to join
 * @param peer_name local peer name to announce on the network, i.e. robot identifier
 * @param port UDP port to listen on for messages
 * @param service_publisher service publisher to announce group membership with
 * @param crypto_key encryption key
 * @param crypto_cipher cipher to use
 */
GossipGroup::GossipGroup(std::string &group_name, std::string &peer_name,
			 std::string &broadcast_address, unsigned short broadcast_port,
			 ServicePublisher *service_publisher,
			 const std::string &crypto_key, const std::string &crypto_cipher)
  : name_(group_name), service_publisher_(service_publisher)
{
  pb_peer_ =
    std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer>(
      new protobuf_comm::ProtobufBroadcastPeer(broadcast_address, broadcast_port,
					       crypto_key, crypto_cipher));

  service_ =
    std::shared_ptr<NetworkService>(new NetworkService(peer_name.c_str(),
						       GOSSIP_MDNSSD_SERVICE_NAME,
						       broadcast_port));

  service_->add_txt("group=%s", group_name.c_str());
  service_publisher_->publish_service(service_.get());
}


/** Constructor.
 * @param group_name name of the group to join
 * @param peer_name local peer name to announce on the network, i.e. robot identifier
 * @param send_port UDP port to send messages to
 * @param recv_port UDP port to listen on for messages
 * @param service_publisher service publisher to announce group membership with
 * @param crypto_key encryption key
 * @param crypto_cipher cipher to use
 */
GossipGroup::GossipGroup(std::string &group_name, std::string &peer_name,
			 std::string &broadcast_address,
			 unsigned short send_port, unsigned short recv_port,
			 ServicePublisher *service_publisher,
			 const std::string &crypto_key, const std::string &crypto_cipher)
  : name_(group_name), service_publisher_(service_publisher)
{
  pb_peer_ =
    std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer>(
      new protobuf_comm::ProtobufBroadcastPeer(broadcast_address, send_port, recv_port,
					       crypto_key, crypto_cipher));

  service_ =
    std::shared_ptr<NetworkService>(new NetworkService(peer_name.c_str(),
						       GOSSIP_MDNSSD_SERVICE_NAME,
						       recv_port));

  service_->add_txt("group=%s", group_name.c_str());
  service_publisher_->publish_service(service_.get());
}


/** Destructor. */
GossipGroup::~GossipGroup()
{
  service_publisher_->unpublish_service(service_.get());
  service_.reset();
  pb_peer_.reset();
}


/** Send a message.
 * @param peer peer to send message to
 * @param m message to send
 */
void
GossipGroup::send(std::string &peer,
		  google::protobuf::Message &m)
{
  pb_peer_->send(m);
}


/** Broadcast a message to all peers in the group.
 * @param m message to send
 */
void
GossipGroup::broadcast(google::protobuf::Message &m)
{
  pb_peer_->send(m);
}


} // end namespace fawkes
