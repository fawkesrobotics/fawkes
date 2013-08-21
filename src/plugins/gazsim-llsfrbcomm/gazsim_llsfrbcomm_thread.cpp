
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

#include "gazsim_llsfrbcomm_thread.h"

#include <protobuf_comm/client.h>

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
  : Thread("GazsimLLSFRbCommThread", Thread::OPMODE_WAITFORWAKEUP)
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
  client_ = new ProtobufStreamClient();

  client_->signal_connected().connect(
    boost::bind(&GazsimLLSFRbCommThread::client_connected, this));
  client_->signal_disconnected().connect(
    boost::bind(&GazsimLLSFRbCommThread::client_disconnected,
		this, boost::asio::placeholders::error));
  client_->signal_received().connect(
    boost::bind(&GazsimLLSFRbCommThread::client_msg, this, _1, _2, _3));

  client_->async_connect("localhost", 4444);
}


void
GazsimLLSFRbCommThread::finalize()
{
}


void
GazsimLLSFRbCommThread::loop()
{
}

void
GazsimLLSFRbCommThread::client_connected()
{
  logger->log_info(name(), "Connected");
}

void
GazsimLLSFRbCommThread::client_disconnected(const boost::system::error_code &error)
{
  logger->log_info(name(), "Disconnected");
}

void
GazsimLLSFRbCommThread::client_msg(uint16_t comp_id, uint16_t msg_type,
			     std::shared_ptr<google::protobuf::Message> msg)
{
  logger->log_info(name(), "Message");
}
