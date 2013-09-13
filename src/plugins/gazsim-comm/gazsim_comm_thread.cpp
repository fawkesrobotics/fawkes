/***************************************************************************
 *  gazsim_comm_plugin.cpp - Plugin simulates peer-to-peer communication over
 *                    an network with configurable instability and manages
 *                    the frowarding of messages to different ports on
 *                    the same machine.
 *
 *  Created: Thu Sep 12 11:09:48 2013
 *  Copyright  2013  Frederik Zwilling
 *
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

#include <aspect/blocked_timing.h>
#include <protobuf_comm/peer.h>
#include <protobuf_comm/message_register.h>

#include "gazsim_comm_thread.h"

using namespace fawkes;
using namespace protobuf_comm;

/** @class GazsimCommThread "clips_thread.h"
 * Plugin simulates and manages communication for Simulation in Gazebo
 * @author Frederik Zwilling
 */

GazsimCommThread::GazsimCommThread()
  : Thread("GazsimCommThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

GazsimCommThread::~GazsimCommThread()
{
}


void
GazsimCommThread::init()
{
  //logger->log_info(name(), "GazsimComm initializing");
  initialized_ = false;

  //read config values
  // address_ = config->get_string("/gazsim/comm/address");
  // send_port_ = config->get_uint("/gazsim/comm/send-port");
  // recv_port_ = config->get_uint("/gazsim/comm/recv-port");
  proto_dirs_ = config->get_strings("/gazsim/proto-dirs");
  addresses_ = config->get_strings("/gazsim/comm/addresses");
  send_ports_ = config->get_uints("/gazsim/comm/send-ports");
  recv_ports_ = config->get_uints("/gazsim/comm/recv-ports");
  if(addresses_.size() != send_ports_.size() || addresses_.size() != recv_ports_.size())
  {
    logger->log_warn(name(), "/gazsim/comm/ has an invalid configuration!");
  }



  //resolve proto paths
  try {
    proto_dirs_ = config->get_strings("/clips-protobuf/proto-dirs");
    for (size_t i = 0; i < proto_dirs_.size(); ++i) {
      std::string::size_type pos;
      if ((pos = proto_dirs_[i].find("@BASEDIR@")) != std::string::npos) {
	proto_dirs_[i].replace(pos, 9, BASEDIR);
      }
      if ((pos = proto_dirs_[i].find("@FAWKES_BASEDIR@")) != std::string::npos) {
	proto_dirs_[i].replace(pos, 16, FAWKES_BASEDIR);
      }
      if ((pos = proto_dirs_[i].find("@RESDIR@")) != std::string::npos) {
	proto_dirs_[i].replace(pos, 8, RESDIR);
      }
      if ((pos = proto_dirs_[i].find("@CONFDIR@")) != std::string::npos) {
	proto_dirs_[i].replace(pos, 9, CONFDIR);
      }
      if (proto_dirs_[i][proto_dirs_.size()-1] != '/') {
	proto_dirs_[i] += "/";
      }
    }
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to load proto paths from config, exception follows");
    logger->log_warn(name(), e);
  }

  //create peer connections 
  peers_.resize(addresses_.size());
  for(unsigned int i = 0; i < addresses_.size(); i++)
  {
    peers_[i] = new ProtobufBroadcastPeer(addresses_[i], send_ports_[i],
					  recv_ports_[i], proto_dirs_);
    peers_[i]->signal_received().connect(boost::bind(&GazsimCommThread::receive_msg, this, _1, _2, _3, _4));
  }
  initialized_ = true;
}


void
GazsimCommThread::finalize()
{
  for(unsigned int i = 0; i < peers_.size(); i++)
  {
    delete peers_[i];
  }
}


void
GazsimCommThread::loop()
{
}

void
GazsimCommThread::receive_msg(boost::asio::ip::udp::endpoint &endpoint,
		       uint16_t component_id, uint16_t msg_type,
		       std::shared_ptr<google::protobuf::Message> msg)
{
  logger->log_info(name(), "Got Peer Message at port %d", endpoint.port());
  logger->log_info(name(), msg->GetTypeName().c_str());
  unsigned int incoming_peer_port = endpoint.port(); //this is suprisingly the send-port
 
  if(!initialized_)
  {
    return;
  }

  //send message to all other peers
  for(unsigned int i = 0; i < peers_.size(); i++)
  {
    if(send_ports_[i] != incoming_peer_port)
    {
      peers_[i]->send(msg);
    }
  }
}
