
/***************************************************************************
 *  gossip_group.h - Robot Group Communication - Gossip Group
 *
 *  Created: Tue Mar 04 10:56:48 2014
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

#ifndef __PLUGINS_GOSSIP_GOSSIP_GOSSIP_GROUP_H_
#define __PLUGINS_GOSSIP_GOSSIP_GOSSIP_GROUP_H_

#include <protobuf_comm/peer.h>

#include <google/protobuf/message.h>

#include <boost/asio.hpp>
#include <boost/signals2.hpp>
#include <memory>

namespace protobuf_comm {
  class ProtobufBroadcastPeer;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ServicePublisher;
class NetworkService;
class GossipGroupManager;

class GossipGroup {
  friend GossipGroupManager;
 public:
  ~GossipGroup();

  void send(std::string &peer,
	    google::protobuf::Message &m);

  void broadcast(google::protobuf::Message &m);



  /** Get the protobuf message register.
   * @return message register */
  protobuf_comm::MessageRegister &  message_register()
  { return pb_peer_->message_register(); }

  /** Signal that is invoked when a message has been received.
   * @return signal */
  boost::signals2::signal<void (boost::asio::ip::udp::endpoint &, uint16_t, uint16_t,
				std::shared_ptr<google::protobuf::Message>)> &
  signal_received()
  { return pb_peer_->signal_received(); }

  /** Signal that is invoked when receiving a message failed.
   * @return signal */
  boost::signals2::signal<void (boost::asio::ip::udp::endpoint &, std::string)> &
  signal_recv_error()
  { return pb_peer_->signal_recv_error(); }

  /** Signal that is invoked when sending a message failed.
   * @return signal */
  boost::signals2::signal<void (std::string)> &
  signal_send_error()
  { return pb_peer_->signal_send_error(); }

  /** Get group name.
   * @return group name. */
  const std::string &  name() const { return name_; }

  /** Get Protobuf broadcast peer.
   * @return protobuf broadcast peer. */
  std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer> peer() const
  { return pb_peer_; }

 private:
  GossipGroup(std::string &group_name, std::string &peer_name,
	      std::string &broadcast_address, unsigned short broadcast_port,
	      ServicePublisher *service_publisher,
	      const std::string &crypto_key, const std::string &crypto_cipher);

  GossipGroup(std::string &group_name, std::string &peer_name,
	      std::string &broadcast_address,
	      unsigned short send_port, unsigned short recv_port,
	      ServicePublisher *service_publisher,
	      const std::string &crypto_key, const std::string &crypto_cipher);

 private:
  std::string     name_;

  ServicePublisher *service_publisher_;
  std::shared_ptr<NetworkService> service_;
  std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer> pb_peer_;
};


} // end namespace fawkes


#endif
