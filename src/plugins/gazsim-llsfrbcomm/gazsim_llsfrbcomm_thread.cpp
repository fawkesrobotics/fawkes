
/***************************************************************************
 *  llsfrbcomm_plugin.cpp - Plugin is a adapter between 
 *       Protobuf Refbox communication and 
 *       Protobuf communication over the gazebo node
 *
 *  Created: Wed Aug 21 15:18:27 2013
 *  Copyright  2013  Frederik Zwilling
 *                   Tim Niemueller [www.niemueller.de]
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
#include <protobuf_comm/client.h>
#include <protobuf_comm/message_register.h>

#include "gazsim_llsfrbcomm_thread.h"
#include <protobuf_msgs/MachineInfo.pb.h>

using namespace fawkes;
using namespace protobuf_comm;

/** @class GazsimLLSFRbCommThread "clips_thread.h"
 * Plugin is a adapter between 
 *       Protobuf Refbox communication and 
 *       Protobuf communication over the gazebo node
 *
 * @author Frederik Zwilling, Tim Niemueller
 */

/** Constructor. */
GazsimLLSFRbCommThread::GazsimLLSFRbCommThread()
  : Thread("GazsimLLSFRbCommThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}


/** Destructor. */
GazsimLLSFRbCommThread::~GazsimLLSFRbCommThread()
{
}


void
GazsimLLSFRbCommThread::init()
{
  //logger->log_info(name(), "GazsimLLSFRbComm initialized");

  //read config values
  refbox_host_ = config->get_string("/gazsim/llsf-rb-comm/refbox-host");
  refbox_port_ = config->get_uint("/gazsim/llsf-rb-comm/refbox-port");
  proto_dirs_ = config->get_strings("/gazsim/llsf-rb-comm/proto-dirs");
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

  //prepare client
  create_client();
  client_->async_connect(refbox_host_.c_str(), refbox_port_);
  //this invokes the connect in the loop
  disconnected_recently_ = true;

  //create publisher and subscriber for connection with gazebo node
  machine_info_pub_ = gazebonode->Advertise<llsf_msgs::MachineInfo>("~/LLSFRbSim/MachineInfo/");
  place_puck_under_machine_sub_ = gazebonode->Subscribe(std::string("~/LLSFRbSim/PlacePuckUnderMachine/"), &GazsimLLSFRbCommThread::on_puck_place_msg, this);
  remove_puck_under_machine_sub_ = gazebonode->Subscribe(std::string("~/LLSFRbSim/RemovePuckFromMachine/"), &GazsimLLSFRbCommThread::on_puck_remove_msg, this);
}


void
GazsimLLSFRbCommThread::finalize()
{
  delete message_register_;
  delete client_;
}


void
GazsimLLSFRbCommThread::loop()
{
  /*if(disconnected_recently_)
  {
    disconnected_recently_ = false;
    //connect
    logger->log_info(name(), "Dong Try");
    client_->async_connect(refbox_host_.c_str(), refbox_port_);
    }*/
}

void
GazsimLLSFRbCommThread::client_connected()
{
  logger->log_info(name(), "Connected to Refbox");
}

void
GazsimLLSFRbCommThread::client_disconnected(const boost::system::error_code &error)
{
  logger->log_info(name(), "Disconnected");
  create_client();
}

void
GazsimLLSFRbCommThread::client_msg(uint16_t comp_id, uint16_t msg_type,
			     std::shared_ptr<google::protobuf::Message> msg)
{
  //logger->log_info(name(), "Message");
  //logger->log_info(name(), msg->GetTypeName().c_str());
  
  //Filter wanted messages
  if(msg->GetTypeName() == "llsf_msgs.MachineInfo")
  {
    //logger->log_info(name(), "Sending MachineInfo to gazebo");
    machine_info_pub_->Publish(*msg);
  }
}

void GazsimLLSFRbCommThread::create_client()
{
  //create message register with all messages to listen for
  message_register_ = new protobuf_comm::MessageRegister(proto_dirs_);

  //create client and register handlers
  client_ = new ProtobufStreamClient(message_register_);
  client_->signal_connected().connect(
    boost::bind(&GazsimLLSFRbCommThread::client_connected, this));
  client_->signal_disconnected().connect(
    boost::bind(&GazsimLLSFRbCommThread::client_disconnected,
		this, boost::asio::placeholders::error));
  client_->signal_received().connect(
    boost::bind(&GazsimLLSFRbCommThread::client_msg, this, _1, _2, _3));

  //this invokes the connect in the loop
  disconnected_recently_ = true;
}

void GazsimLLSFRbCommThread::on_puck_place_msg(ConstPlacePuckUnderMachinePtr &msg)
{
  //logger->log_info(name(), "Sending PPUM to refbox");
  if(!client_->connected())
  {
    return;
  }
  llsf_msgs::PlacePuckUnderMachine to_rb = *msg;
  client_->send(to_rb);
}

void GazsimLLSFRbCommThread::on_puck_remove_msg(ConstRemovePuckFromMachinePtr &msg)
{
  //logger->log_info(name(), "Sending RPFM to refbox");
  if(!client_->connected())
  {
    return;
  }
  llsf_msgs::RemovePuckFromMachine to_rb = *msg;
  client_->send(to_rb);
}
