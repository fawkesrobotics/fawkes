
/***************************************************************************
 *  backend_thread.h - World Info Viewer backend thread
 *
 *  Created: Thu April 10 21:53:24 2008
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

#ifndef __TOOL_WORLDINFO_VIEWER_BACKEND_THREAD_H_
#define __TOOL_WORLDINFO_VIEWER_BACKEND_THREAD_H_

#include <netcomm/worldinfo/handler.h>
#include <netcomm/utils/resolver.h>
#include <netcomm/dns-sd/avahi_thread.h>
#include <core/threading/thread.h>

#include <gtkmm.h>
#include <string>

namespace fawkes {
  class WorldInfoTransceiver;
  class WorldInfoDataContainer;
}

class WorldInfoViewerBackendThread
: public fawkes::Thread,
  public fawkes::WorldInfoHandler
{
 public:
  WorldInfoViewerBackendThread( fawkes::WorldInfoDataContainer* data_container,
				const char* addr, unsigned short port,
				const char* key, const char* iv );

  virtual ~WorldInfoViewerBackendThread();

  Glib::Dispatcher& new_worldinfo_data();
  Glib::Dispatcher& new_gamestate_data();

  // thread
  void loop();

  // handler
  virtual void pose_rcvd( const char *from_host,
			  float x, float y, float theta,
			  float *covariance );

  virtual void velocity_rcvd( const char *from_host, float vel_x,
			      float vel_y, float vel_theta, float *covariance );

  virtual void ball_pos_rcvd( const char *from_host,
			      bool visible, int visibility_history,
			      float dist, float pitch, float yaw,
			      float *covariance );

  virtual void ball_velocity_rcvd( const char *from_host,
				   float vel_x, float vel_y, float vel_z,
				   float *covariance );

  virtual void opponent_pose_rcvd( const char *from_host, unsigned int uid,
				   float distance, float angle,
				   float *covarianceconst );
  
  virtual void opponent_disapp_rcvd( const char *from_host, unsigned int uid );

  virtual void gamestate_rcvd( const char *from_host,
			       fawkes::worldinfo_gamestate_t game_state, 
			       fawkes::worldinfo_gamestate_team_t state_team, 
			       unsigned int score_cyan, unsigned int score_magenta, 
			       fawkes::worldinfo_gamestate_team_t our_team, 
			       fawkes::worldinfo_gamestate_goalcolor_t our_goal_color,
			       fawkes::worldinfo_gamestate_half_t half );

 private:
  fawkes::WorldInfoTransceiver*   m_transceiver;
  fawkes::WorldInfoDataContainer* m_data_container;

  Glib::Dispatcher m_signal_new_worldinfo_data;
  Glib::Dispatcher m_signal_new_gamestate_data;

  std::string    m_addr;
  unsigned short m_port;
  std::string    m_key;
  std::string    m_iv;

  fawkes::NetworkNameResolver* m_resolver;
  fawkes::AvahiThread*         m_avahi;

};

#endif /* __TOOL_WORLDINFO_VIEWER_BACKEND_THREAD_H_ */
