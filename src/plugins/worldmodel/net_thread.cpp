
/***************************************************************************
 *  net_thread.cpp - Fawkes WorldModel Plugin Network Thread
 *
 *  Created: Fri Jun 29 16:56:15 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <plugins/worldmodel/net_thread.h>

#include <netcomm/worldinfo/transceiver.h>

#include <string>

using namespace std;

/** @class WorldModelNetworkThread <plugins/worldmodel/net_thread.h>
 * Network thread of worldmodel plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 */
WorldModelNetworkThread::WorldModelNetworkThread()
  : Thread("WorldModelNetworkThread", Thread::OPMODE_CONTINUOUS)
{
  worldinfo_transceiver = NULL;
}


/** Destructor. */
WorldModelNetworkThread::~WorldModelNetworkThread()
{
}


void
WorldModelNetworkThread::init()
{
  logger->log_info(name(), "init() called");
  std::string multicast_addr;
  unsigned int port;
  std::string encryption_key;
  std::string encryption_iv;
  try {
    multicast_addr = config->get_string("/worldmodel/worldinfo_transceiver/multicast_addr");
    port = config->get_uint("/worldmodel/worldinfo_transceiver/port");
    encryption_key = config->get_string("/worldmodel/worldinfo_transceiver/encryption_key");
    encryption_iv  = config->get_string("/worldmodel/worldinfo_transceiver/encryption_iv");
    sleep_time_msec = config->get_uint("/worldmodel/worldinfo_transceiver/sleep_time_msec");
    max_msgs_per_recv = config->get_uint("/worldmodel/worldinfo_transceiver/max_msgs_per_recv");
  } catch (Exception &e) {
    e.append("Could not get required configuration data for worldmodel");
    throw;
  }

  worldinfo_transceiver = new WorldInfoTransceiver(multicast_addr.c_str(), port,
						   encryption_key.c_str(), encryption_iv.c_str(),
						   nnresolver);

  worldinfo_transceiver->add_handler(this);
}


void
WorldModelNetworkThread::finalize()
{
  logger->log_info(name(), "finalize() called");
  delete worldinfo_transceiver;
}


void
WorldModelNetworkThread::loop()
{
  worldinfo_transceiver->recv(true, max_msgs_per_recv);
  usleep( sleep_time_msec * 1000 );
}


void
WorldModelNetworkThread::pose_rcvd(const char *from_host,
				   float x, float y, float theta,
				   float *covariance)
{
}


void
WorldModelNetworkThread::velocity_rcvd(const char *from_host, float vel_x,
				       float vel_y, float vel_theta, float *covariance)
{
}


void
WorldModelNetworkThread::ball_pos_rcvd(const char *from_host,
				       float dist, float pitch, float yaw,
				       float *covariance)
{
}


void
WorldModelNetworkThread::ball_velocity_rcvd(const char *from_host,
					    float vel_x, float vel_y, float vel_z,
					    float *covariance)
{
}


void
WorldModelNetworkThread::opponent_pose_rcvd(const char *from_host,
					    float distance, float angle,
					    float *covariance)
{
}
