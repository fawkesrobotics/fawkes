
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
 *  (at your option) any later version.
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
#include <worldinfo_utils/data_container.h>

#include <string>

using namespace std;
using namespace fawkes;

/** @class WorldModelNetworkThread net_thread.h <plugins/worldmodel/net_thread.h>
 * Network thread of worldmodel plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 */
WorldModelNetworkThread::WorldModelNetworkThread()
  : Thread("WorldModelNetworkThread", Thread::OPMODE_CONTINUOUS)
{
  data = 0;
  worldinfo_transceiver = NULL;
  set_prepfin_conc_loop(true);
}


/** Destructor. */
WorldModelNetworkThread::~WorldModelNetworkThread()
{
}


/** Gain access to the worldinfo transceiver.
 * @return pointer to the worldinfo transceiver
 */
WorldInfoTransceiver *
WorldModelNetworkThread::get_transceiver()
{
  return worldinfo_transceiver;
}


/** Gain access to the worldinfo data container.
 * @return pointer to the worldinfo data container
 */
WorldInfoDataContainer *
WorldModelNetworkThread::get_data_container()
{
  return data;
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
    multicast_addr = config->get_string("/worldinfo/multicast_addr");
    port = config->get_uint("/worldinfo/udp_port");
    encryption_key = config->get_string("/worldinfo/encryption_key");
    encryption_iv  = config->get_string("/worldinfo/encryption_iv");
    sleep_time_msec = 100; //config->get_uint("/worldinfo/sleep_time_msec");
    max_msgs_per_recv = 10; //config->get_uint("/worldinfo/max_msgs_per_recv");
  } catch (Exception &e) {
    e.append("Could not get required configuration data for worldmodel");
    throw;
  }

  worldinfo_transceiver = new WorldInfoTransceiver(multicast_addr.c_str(), port,
						   encryption_key.c_str(), encryption_iv.c_str(),
						   nnresolver);

  worldinfo_transceiver->add_handler(this);
  // DEBUG
  worldinfo_transceiver->set_loop(true);

  data = new WorldInfoDataContainer(clock);
}


void
WorldModelNetworkThread::finalize()
{
  logger->log_info(name(), "finalize() called");
  delete worldinfo_transceiver;
  delete data;
}


void
WorldModelNetworkThread::loop()
{
  worldinfo_transceiver->recv(false, max_msgs_per_recv);
  usleep( sleep_time_msec * 1000 );
}


void
WorldModelNetworkThread::pose_rcvd(const char *from_host,
				   float x, float y, float theta,
				   float *covariance)
{
  data->set_robot_pose(from_host, x, y, theta, covariance);
}


void
WorldModelNetworkThread::velocity_rcvd(const char *from_host, float vel_x,
				       float vel_y, float vel_theta, float *covariance)
{
  // TODO
}


void
WorldModelNetworkThread::ball_pos_rcvd(const char *from_host,
				       bool visible, int visibility_history,
				       float dist, float bearing, float slope,
				       float *covariance)
{
  // TODO: visible, visibility_history
  if ( visible )
    { data->set_ball_pos(from_host, dist, bearing, slope, covariance); }
  else
    { data->delete_ball_pos(from_host); }
}


void
WorldModelNetworkThread::ball_velocity_rcvd(const char *from_host,
					    float vel_x, float vel_y, float vel_z,
					    float *covariance)
{
  // TODO
}


void
WorldModelNetworkThread::opponent_pose_rcvd(const char *from_host,
					    unsigned int uid,
					    float distance, float bearing,
					    float *covariance)
{
  data->set_opponent_pos(from_host, uid, distance, bearing, covariance);
}


void
WorldModelNetworkThread::opponent_disapp_rcvd(const char *from_host, unsigned int uid)
{
  // TODO
}


void
WorldModelNetworkThread::gamestate_rcvd(const char *from_host,
					fawkes::worldinfo_gamestate_t game_state,
					fawkes::worldinfo_gamestate_team_t state_team,
					unsigned int score_cyan, unsigned int score_magenta,
					fawkes::worldinfo_gamestate_team_t our_team,
					fawkes::worldinfo_gamestate_goalcolor_t our_goal_color,
					fawkes::worldinfo_gamestate_half_t half)
{
  data->set_game_state(game_state, state_team, score_cyan, score_magenta, 
		       our_team, our_goal_color, half);
}
