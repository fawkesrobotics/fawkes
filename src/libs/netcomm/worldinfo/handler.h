
/***************************************************************************
 *  handler.h - World Info Handler
 *
 *  Created: Sun Jan 14 18:07:04 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __NETCOMM_WORLDINFO_HANDLER_H_
#define __NETCOMM_WORLDINFO_HANDLER_H_

class WorldInfoHandler
{
 public:
  virtual ~WorldInfoHandler();

  virtual void pose_rcvd(const char *from_host,
			 float x, float y, float theta,
			 float *covariance)                                     = 0;

  virtual void velocity_rcvd(const char *from_host, float vel_x,
			     float vel_y, float vel_theta, float *covariance)   = 0;

  virtual void ball_pos_rcvd(const char *from_host,
			     float dist, float pitch, float yaw,
			     float *covariance)                                 = 0;

  virtual void ball_velocity_rcvd(const char *from_host,
				  float vel_x, float vel_y, float vel_z,
				  float *covariance)                            = 0;

  virtual void opponent_pose_rcvd(const char *from_host,
				  float distance, float angle,
				  float *covariance)                            = 0;
};


#endif
