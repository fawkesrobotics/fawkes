
/***************************************************************************
 *  velocitymodel.h - Abstract class defining a velocity model
 *
 *  Created: Mon Sep 05 16:59:58 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __FIREVISION_VELOCITYMODEL_H_
#define __FIREVISION_VELOCITYMODEL_H_

#include <sys/time.h>
#include <fvutils/base/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class VelocityModel
{

 public:
  virtual ~VelocityModel();

  virtual const char *  getName() const                                = 0;

  virtual void	setPanTilt(float pan, float tilt)                          = 0;
  virtual void  setRobotPosition(float x, float y, float ori, timeval t)   = 0;
  virtual void  setRobotVelocity(float vel_x, float vel_y, timeval t)      = 0;
  virtual void  setTime(timeval t)                                         = 0;
  virtual void  setTimeNow()                                               = 0;
  virtual void  getTime(long int *sec, long int *usec)                     = 0;

  /* Method to retrieve velocity information
   * @param vel_x If not NULL contains velocity in X direction after call
   * @param vel_y If not NULL contains velocity in Y direction after call
   */
  virtual void  getVelocity(float *vel_x, float *vel_y)              = 0;

  virtual float getVelocityX()                                       = 0;
  virtual float getVelocityY()                                       = 0;


  /** Calculate velocity values from given data
   * This method must be called after all relevent data (set*) has been
   * set. After calc() the velocity values can be retrieved
   */
  virtual void calc()                                               = 0;

  /** Reset velocity model
   * Must be called if ball is not visible at any time
   */
  virtual void reset()                                              = 0;

  /** Returns the used coordinate system, must be either COORDSYS_ROBOT_CART or
   * COORDSYS_ROBOT_WORLD. ROBOT denotes velocities relative to the robot
   * (which can be tramsformed to global velocities by:
   * glob_vel_x = rel_vel_x * cos( robot_ori ) - rel_vel_y * sin( robot_ori )
   * WORLD denotes velocities in the robot coordinate system
   * glob_vel_y = rel_vel_x * sin( robot_ori ) + rel_vel_y * cos( robot_ori )
   * @return coordinate system type
   */
  virtual coordsys_type_t getCoordinateSystem()                     = 0;

};

} // end namespace firevision

#endif
