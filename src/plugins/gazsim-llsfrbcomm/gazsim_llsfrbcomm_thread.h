
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

#ifndef __PLUGINS_GAZSIM_LLSFRBCOMM_LLSFRBCOMM_THREAD_H_
#define __PLUGINS_GAZSIM_LLSFRBCOMM_LLSFRBCOMM_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blocked_timing.h>
#include <boost/asio.hpp>
#include <google/protobuf/message.h>
#include <protobuf_comm/client.h>
#include <protobuf_comm/message_register.h>
#include <plugins/gazebo/aspect/gazebo.h>
#include <protobuf_msgs/MachineInfo.pb.h>
#include <protobuf_msgs/MachineCommands.pb.h>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>

typedef const boost::shared_ptr<llsf_msgs::MachineInfo const> ConstMachineInfoPtr;
typedef const boost::shared_ptr<llsf_msgs::PlacePuckUnderMachine const> ConstPlacePuckUnderMachinePtr;
typedef const boost::shared_ptr<llsf_msgs::RemovePuckFromMachine const> ConstRemovePuckFromMachinePtr;



namespace protobuf_comm {
  class ProtobufStreamClient;
}

class GazsimLLSFRbCommThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::GazeboAspect,
  public fawkes::LoggingAspect
{
 public:
  GazsimLLSFRbCommThread();
  ~GazsimLLSFRbCommThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  void client_connected();
  void client_disconnected(const boost::system::error_code &error);
  void client_msg(uint16_t comp_id, uint16_t msg_type,
		  std::shared_ptr<google::protobuf::Message> msg);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  protobuf_comm::ProtobufStreamClient *client_;
  protobuf_comm::MessageRegister      *message_register_;

  //config values
  std::vector<std::string> proto_dirs_;
  std::string refbox_host_;
  unsigned int refbox_port_;

  //Publisher and subscriber for the connection to gazebo
  gazebo::transport::PublisherPtr machine_info_pub_;
  gazebo::transport::SubscriberPtr place_puck_under_machine_sub_;
  gazebo::transport::SubscriberPtr remove_puck_under_machine_sub_;

  //handler methods
  void on_puck_place_msg(ConstPlacePuckUnderMachinePtr &msg);
  void on_puck_remove_msg(ConstRemovePuckFromMachinePtr &msg);

  //helper variables
  bool disconnected_recently_;

  void create_client();
};

#endif
