
/***************************************************************************
 *  net_thread.h - Fawkes WorldModel Plugin Network Thread
 *
 *  Created: Fri Jun 29 16:55:52 2007 (on flight to RoboCup 2007, Atlanta)
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

#ifndef __PLUGINS_WORLDMODEL_NET_THREAD_H_
#define __PLUGINS_WORLDMODEL_NET_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/network.h>
#include <netcomm/worldinfo/handler.h>

class WorldInfoTransceiver;

class WorldModelNetworkThread
  : public Thread,
    public LoggingAspect,
    public ConfigurableAspect,
    public ClockAspect,
    public NetworkAspect,
    public WorldInfoHandler
{
 public:
  WorldModelNetworkThread();
  virtual ~WorldModelNetworkThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  /* WorldInfoHandler methods follow */
  virtual void pose_rcvd(const char *from_host,
			 float x, float y, float theta,
			 float *covariance);

  virtual void velocity_rcvd(const char *from_host, float vel_x,
			     float vel_y, float vel_theta, float *covariance);

  virtual void ball_pos_rcvd(const char *from_host,
			     float dist, float pitch, float yaw,
			     float *covariance);

  virtual void ball_velocity_rcvd(const char *from_host,
				  float vel_x, float vel_y, float vel_z,
				  float *covariance);

  virtual void opponent_pose_rcvd(const char *from_host,
				  float distance, float angle,
				  float *covariance);

 private:
  WorldInfoTransceiver *worldinfo_transceiver;

  unsigned int sleep_time_msec;
  unsigned int max_msgs_per_recv;

};


#endif
