
/***************************************************************************
 *  backend_thread.cpp - World Info Viewer backend thread
 *
 *  Created: Thu April 10 22:00:08 2008
 *  Copyright  2008  Daniel Beck
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

#include "backend_thread.h"
#include <worldinfo_utils/data_container.h>
#include <netcomm/worldinfo/transceiver.h>

using namespace fawkes;

/** @class WorldInfoViewerBackendThread backend_thread.h <tools/worldinfo_viewer/backend_thread.h>
 * The backend thread of the worldinfo viewer application.
 * @author Daniel Beck
 */

/** Constructor.  
 * @param data_container pointer to the central instance of the
 * WorldInfoDataContainer
 * @param addr multicast address to use for worldinfo communication
 * @param port port
 * @param key de-/encryption key
 * @param iv initialization vector for de-/encryption
 */ 
WorldInfoViewerBackendThread::WorldInfoViewerBackendThread( WorldInfoDataContainer* data_container,
							    const char* addr,
							    unsigned short port,
							    const char* key,
							    const char* iv )
  : Thread("WorldInfoViewerBackendThread")
{
  m_data_container = data_container;

  m_addr = addr;
  m_port = port;
  m_key  = key;
  m_iv   = iv;

  m_avahi = new AvahiThread();
  m_avahi->start();

  m_resolver = new NetworkNameResolver( m_avahi );

  m_transceiver = new WorldInfoTransceiver( m_addr.c_str(),
					    m_port, 
					    m_key.c_str(),
					    m_iv.c_str(),
					    m_resolver );
  m_transceiver->add_handler(this);
}

/** Destructor. */
WorldInfoViewerBackendThread::~WorldInfoViewerBackendThread()
{
  delete m_transceiver;
  delete m_resolver;

  m_avahi->cancel();
  m_avahi->join();
  delete m_avahi;
}

/** Access the dispatcher that is emitted whenever new data has
 * arrived.
 * @return reference to the dispatcher
 */
Glib::Dispatcher&
WorldInfoViewerBackendThread::new_worldinfo_data()
{
  return m_signal_new_worldinfo_data;
}

/** Access the dispatcher that is emitted whenever new game state data
 * has arrived.
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
  m_transceiver->flush_sequence_numbers( 10 );
  m_transceiver->recv(true, 100);
  usleep(100000);
}

void
WorldInfoViewerBackendThread::pose_rcvd( const char* from_host,
					 float x,
					 float y,
					 float theta,
					 float* covariance )
{
#ifdef DEBUG_PRINT
  printf( "Received pose data from host %s: x=%.3f y=%.3f theta=%.3f\n",
	  from_host, x, y, theta );
#endif /* DEBUG_PRING */
  
  m_data_container->set_robot_pose( from_host, x, y, theta, covariance );
  m_signal_new_worldinfo_data();
}

void
WorldInfoViewerBackendThread::velocity_rcvd( const char* from_host,
					     float vel_x,
					     float vel_y,
					     float vel_theta,
					     float* covariance )
{
#ifdef DEBUG_PRINT
  printf( "Received velocity data from host %s: vx=%.3f vy=%.3f vtheta=%.3f\n",
	  from_host, vel_x, vel_y, vel_theta );
#endif /* DEBUG_PRINT */
  
  m_data_container->set_robot_velocity( from_host, vel_x, vel_y, vel_theta,
					covariance );
  m_signal_new_worldinfo_data();
}

void
WorldInfoViewerBackendThread::ball_pos_rcvd( const char* from_host,
					     bool visible,
					     int visibility_history,
					     float dist,
					     float bearing,
					     float slope,
					     float* covariance )
{
#ifdef DEBUG_PRINT
  if ( visible )
  { printf( "Received ball data from host %s: dist=%.3f bearing=%.3f\n",
	    from_host, dist, bearing ); }
  else
  { printf( "Received ball not visible from host %s\n", from_host ); }
#endif /* DEBUG_PRINT */
  
  m_data_container->set_ball_pos( from_host, visible, visibility_history,
				  dist, bearing, slope, covariance );
  m_signal_new_worldinfo_data();
}

void
WorldInfoViewerBackendThread::global_ball_pos_rcvd( const char* from_host,
						    bool visible,
						    int visibility_history,
						    float x,
						    float y,
						    float z,
						    float* covariance )
{
#ifdef DEBUG_PRINT
  if ( visible )
  { printf( "Received global ball data from host %s: x=%.3f y=%.3f\n",
	    from_host, x, y ); }
  else
  { printf( "Received global ball not visible from host %s\n", from_host ); }
#endif /* DEBUG_PRINT */
  m_data_container->set_ball_pos_global( from_host, visible,
					 visibility_history,
					 x, y, z, covariance );
  m_signal_new_worldinfo_data();
}

void
WorldInfoViewerBackendThread::ball_velocity_rcvd( const char* from_host,
						  float vel_x,
						  float vel_y,
						  float vel_z,
						  float* covariance )
{
  m_data_container->set_ball_velocity( from_host, vel_x, vel_y, vel_z,
				       covariance );
  m_signal_new_worldinfo_data();
}

void
WorldInfoViewerBackendThread::global_ball_velocity_rcvd( const char *from_host,
							 float vel_x,
							 float vel_y,
							 float vel_z,
							 float *covariance )
{
  // TODO

//   m_data_container->set_global_ball_velocity( from_host, vel_x, vel_y, vel_z,
// 					      covariance );
//   m_signal_new_worldinfo_data();
}


void
WorldInfoViewerBackendThread::opponent_pose_rcvd( const char* from_host,
						  unsigned int uid,
						  float distance,
						  float angle,
						  float* covariance )
{
// #ifdef DEBUG_PRINT
//   printf("Received opponent pose data form host %s\n", from_host );
// #endif /* DEBUG_PRINT */

  m_data_container->set_opponent_pos( from_host, uid, distance, angle,
				      covariance );
  m_signal_new_worldinfo_data();
}


void
WorldInfoViewerBackendThread::opponent_disapp_rcvd( const char *from_host,
						    unsigned int uid )
{
  m_data_container->opponent_disappeared( from_host, uid );
  m_signal_new_worldinfo_data();
}


void
WorldInfoViewerBackendThread::gamestate_rcvd( const char* from_host, 
					      unsigned int game_state, 
					      worldinfo_gamestate_team_t state_team, 
					      unsigned int score_cyan, 
					      unsigned int score_magenta, 
					      worldinfo_gamestate_team_t own_team, 
					      worldinfo_gamestate_goalcolor_t own_goal_color,
					      worldinfo_gamestate_half_t half )
{
#ifdef DEBUG_PRINT
  printf( "Received gamestate data from host %s\n", from_host );
#endif /* DEBUG_PRINT */

  m_data_container->set_game_state( game_state, state_team,
				    score_cyan, score_magenta,
				    own_team, own_goal_color, half );
  m_signal_new_gamestate_data();
}

void
WorldInfoViewerBackendThread::penalty_rcvd(const char *from_host,
					   unsigned int player,
					   unsigned int penalty,
					   unsigned int seconds_remaining)
{
}
