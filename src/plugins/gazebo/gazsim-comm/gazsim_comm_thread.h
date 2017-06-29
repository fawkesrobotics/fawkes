/***************************************************************************
 *  gazsim_comm_plugin.cpp - Plugin simulates peer-to-peer communication over
 *                    an network with configurable instability and manages
 *                    the frowarding of messages to different ports on
 *                    the same machine.
 *
 *  Created: Thu Sep 12 11:07:43 2013
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

#ifndef __PLUGINS_GAZSIM_COMM_COMM_THREAD_H_
#define __PLUGINS_GAZSIM_COMM_COMM_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blocked_timing.h>
#include <boost/asio.hpp>
#include <google/protobuf/message.h>
#include <protobuf_comm/peer.h>
#include <protobuf_comm/message_register.h>
#include <list>


namespace protobuf_comm {
  class ProtobufStreamClient;
}

class GazsimCommThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect
{
 public:
  GazsimCommThread();
  ~GazsimCommThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void peer_send_error(std::string address, unsigned int port, std::string err);
  void receive_raw_msg(boost::asio::ip::udp::endpoint &endpoint,
		       protobuf_comm::frame_header_t &header, void * data,
		       size_t length);
  
 private:
  std::vector<protobuf_comm::ProtobufBroadcastPeer*> peers_;
  std::vector<protobuf_comm::ProtobufBroadcastPeer*> peers_crypto1_;
  std::vector<protobuf_comm::ProtobufBroadcastPeer*> peers_crypto2_;

  //config values
  std::vector<std::string> addresses_;
  std::vector<unsigned int> send_ports_;
  std::vector<unsigned int> recv_ports_;
  std::vector<unsigned int> send_ports_crypto1_;
  std::vector<unsigned int> recv_ports_crypto1_;
  std::vector<unsigned int> send_ports_crypto2_;
  std::vector<unsigned int> recv_ports_crypto2_;
  
  bool use_crypto1_, use_crypto2_;

  std::vector<std::string> proto_dirs_;
  double package_loss_;

  //helper variables
  bool initialized_;
};

#endif
