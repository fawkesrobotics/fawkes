
/***************************************************************************
 *  backend_thread.cpp - World Info Viewer backend thread
 *
 *  Created: Thu April 10 22:00:08 2008
 *  Copyright  2008  Daniel Beck
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

#include <tools/worldinfo_viewer/backend_thread.h>
#include <worldinfo_utils/data_container.h>
#include <netcomm/worldinfo/transceiver.h>

/** @class WorldInfoViewerBackendThread <tools/worldinfo_viewer/backend_thread.h>
 * The backend thread of the worldinfo viewer application.
 * @author Daniel Beck
 */

/** Constructor.
 * @param data_container pointer to the central instance of the WorldInfoDataContainer
 * @param addr multicast address to use for worldinfo communication
 * @param port port
 * @param key de-/encryption key
 * @param iv initialization vector for de-/encryption
 */ 
WorldInfoViewerBackendThread::WorldInfoViewerBackendThread( WorldInfoDataContainer* data_container,
							    const char* addr, unsigned short port,
							    const char* key, const char* iv )
  : Thread("WorldInfoViewerBackendThread")
{
  m_data_container = data_container;
  m_addr = addr;
  m_port = port;
  m_key = key;
  m_iv = iv;

  m_transceiver = new WorldInfoTransceiver( m_addr.c_str(), m_port, 
					    m_key.c_str(), m_iv.c_str() );
  m_transceiver->add_handler(this);
}

/** Destructor. */
WorldInfoViewerBackendThread::~WorldInfoViewerBackendThread()
{
  delete m_transceiver;
}

/** Access the dispatcher that is emitted whenever new data has arrived.
 * @return reference to the dispatcher
 */
Glib::Dispatcher&
WorldInfoViewerBackendThread::new_data()
{
  return m_signal_new_data;
}

/** Access the dispatcher that is emitted whenever new game state data has arrived.
 * @return reference to the dispatcher
 */
Glib::Dispatcher&
WorldInfoViewerBackendThread::new_gamestate_data()
{
  return m_signal_new_gamestate_data;
}

void
WorldInfoViewerBackendThread::loop()
{
  m_transceiver->recv(true, 100);
}

void
WorldInfoViewerBackendThread::pose_rcvd( const char* from_host,
					 float x, float y, float theta,
					 float* covariance )
{
  m_data_container->set_robot_pose(from_host, x, y, theta, covariance);
  m_signal_new_data();
}

void
WorldInfoViewerBackendThread::velocity_rcvd( const char* from_host, float vel_x,
					     float vel_y, float vel_theta, float* covariance )
{
  // TODO
}

void
WorldInfoViewerBackendThread::ball_pos_rcvd( const char* from_host,
					     bool visible, int visibility_history,
					     float dist, float pitch, float yaw,
					     float* covariance )
{
  m_data_container->set_ball_pos(from_host, dist, pitch, yaw, covariance);
  m_signal_new_data();
}

void
WorldInfoViewerBackendThread::ball_velocity_rcvd( const char* from_host,
						  float vel_x, float vel_y, float vel_z,
						  float* covariance )
{
  // TODO
}

void
WorldInfoViewerBackendThread::opponent_pose_rcvd( const char* from_host, unsigned int uid,
						  float distance, float angle,
						  float* covariance )
{
  // TODO
}


void
WorldInfoViewerBackendThread::opponent_disapp_rcvd(const char *from_host, unsigned int uid)
{
}


void
WorldInfoViewerBackendThread::gamestate_rcvd( const char* from_host, 
					      worldinfo_gamestate_t game_state, 
					      worldinfo_gamestate_team_t state_team, 
					      unsigned int score_cyan, 
					      unsigned int score_magenta, 
					      worldinfo_gamestate_team_t own_team, 
					      worldinfo_gamestate_goalcolor_t own_goal_color,
					      worldinfo_gamestate_half_t half )
{
  m_data_container->set_game_state( game_state, state_team, score_cyan, score_magenta,
				    own_team, own_goal_color, half );
  m_signal_new_gamestate_data();
}
