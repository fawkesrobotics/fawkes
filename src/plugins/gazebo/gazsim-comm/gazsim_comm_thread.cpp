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
#include <stdlib.h>
#include "gazsim_comm_thread.h"
#include <algorithm>

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
  proto_dirs_ = config->get_strings("/gazsim/proto-dirs");
  package_loss_ = config->get_float("/gazsim/comm/package-loss");
  addresses_ = config->get_strings("/gazsim/comm/addresses");
  send_ports_ = config->get_uints("/gazsim/comm/send-ports");
  recv_ports_ = config->get_uints("/gazsim/comm/recv-ports");
  use_crypto1_ = config->get_bool("/gazsim/comm/use-crypto1");
  use_crypto2_ = config->get_bool("/gazsim/comm/use-crypto1");
  send_ports_crypto1_ = config->get_uints("/gazsim/comm/send-ports-crypto1");
  recv_ports_crypto1_ = config->get_uints("/gazsim/comm/recv-ports-crypto1");
  send_ports_crypto2_ = config->get_uints("/gazsim/comm/send-ports-crypto2");
  recv_ports_crypto2_ = config->get_uints("/gazsim/comm/recv-ports-crypto2");
  if(addresses_.size() != send_ports_.size() || addresses_.size() != recv_ports_.size()
     || ( use_crypto1_ && addresses_.size() != send_ports_crypto1_.size())
     || ( use_crypto1_ && addresses_.size() != recv_ports_crypto1_.size())
     || ( use_crypto2_ && addresses_.size() != send_ports_crypto2_.size())
     || ( use_crypto2_ && addresses_.size() != recv_ports_crypto2_.size()))
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
  peers_crypto1_.resize(addresses_.size());
  peers_crypto2_.resize(addresses_.size());
  for(unsigned int i = 0; i < addresses_.size(); i++)
  {
    peers_[i] = new ProtobufBroadcastPeer(addresses_[i], send_ports_[i],
					  recv_ports_[i], proto_dirs_);
    peers_[i]->signal_received_raw().connect(boost::bind(&GazsimCommThread::receive_raw_msg, this, _1, _2, _3, _4));
    peers_[i]->signal_send_error().connect(boost::bind(&GazsimCommThread::peer_send_error, this, addresses_[i], send_ports_[i], _1));
    if(use_crypto1_)
    {
      peers_crypto1_[i] = new ProtobufBroadcastPeer(addresses_[i], send_ports_crypto1_[i],
					  recv_ports_crypto1_[i], proto_dirs_);
      peers_crypto1_[i]->signal_received_raw().connect(boost::bind(&GazsimCommThread::receive_raw_msg, this, _1, _2, _3, _4));
      peers_crypto1_[i]->signal_send_error().connect(boost::bind(&GazsimCommThread::peer_send_error, this, addresses_[i], send_ports_crypto1_[i], _1));
    }
    if(use_crypto2_)
    {
      peers_crypto2_[i] = new ProtobufBroadcastPeer(addresses_[i], send_ports_crypto2_[i],
						    recv_ports_crypto2_[i], proto_dirs_);
      peers_crypto2_[i]->signal_received_raw().connect(boost::bind(&GazsimCommThread::receive_raw_msg, this, _1, _2, _3, _4));
      peers_crypto2_[i]->signal_send_error().connect(boost::bind(&GazsimCommThread::peer_send_error, this, addresses_[i], send_ports_crypto2_[i], _1));
    }
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

/**
 * Receive and forward raw msg
 * @param endpoint port msg received from
 * @param header header of the msg
 * @param data data stream
 * @param length length of the data stream
 */
void
GazsimCommThread::receive_raw_msg(boost::asio::ip::udp::endpoint &endpoint,
				  protobuf_comm::frame_header_t &header, void * data,
				  size_t length)
{
  //logger->log_info(name(), "Got raw Message from port %d", endpoint.port());
  unsigned int incoming_peer_port = endpoint.port(); //this is suprisingly the send-port
 
  if(!initialized_)
  {
    return;
  }

  //simulate package loss
  double rnd = ((double) rand()) / ((double) RAND_MAX); //0.0 <= rnd <= 1.0
  if(rnd < package_loss_)
  {
    return;
  }

  //check which set of peers the message comes from
  std::vector<protobuf_comm::ProtobufBroadcastPeer*> peers;
  std::vector<unsigned int> send_ports;
  if(std::find(send_ports_.begin(), send_ports_.end(), incoming_peer_port) != send_ports_.end())
  {
    peers = peers_;
    send_ports = send_ports_;
  }
  else if(use_crypto1_ && std::find(send_ports_crypto1_.begin(), send_ports_crypto1_.end(), incoming_peer_port) != send_ports_crypto1_.end())
  {
    peers = peers_crypto1_;
    send_ports = send_ports_crypto1_;
  }  
  else if(use_crypto2_ && std::find(send_ports_crypto2_.begin(), send_ports_crypto2_.end(), incoming_peer_port) != send_ports_crypto1_.end())
  {
    peers = peers_crypto2_;
    send_ports = send_ports_crypto2_;
  }

  //send message to all other peers
  for(unsigned int i = 0; i < peers.size(); i++)
  {
    if(send_ports[i] != incoming_peer_port)
    {
      peers[i]->send_raw(header, data, length);
    }
  }
}

void
GazsimCommThread::peer_send_error(std::string address, unsigned int port, std::string err)
{
	logger->log_warn(name(), "Peer send error for %s:%u: %s", address.c_str(), port, err.c_str());
}
