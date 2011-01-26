
/***************************************************************************
 *  velocitymodel.cpp - Abstract class defining a velocity model
 *
 *  Created: Thu Mar 29 17:02:09 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvmodels/velocity/velocitymodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class VelocityModel <fvmodels/velocity/velocitymodel.h>
 * Velocity model interface.
 *
 *
 * @fn const char *  VelocityModel::getName() const
 * Get name of velocity model
 * @return name of velocity model
 *
 * @fn void	VelocityModel::setPanTilt(float pan, float tilt)
 * Set pan and tilt.
 * @param pan pan
 * @param tilt tilt
 *
 * @fn void  VelocityModel::setRobotPosition(float x, float y, float ori, timeval t)
 * Set robot position.
 * @param x x
 * @param y y
 * @param ori ori
 * @param t timestamp of the pose information
 *
 * @fn void  VelocityModel::setRobotVelocity(float vel_x, float vel_y, timeval t)
 * Set robot velocity.
 * @param vel_x robot velocity in x direction
 * @param vel_y robot velocity in y direction
 * @param t timestamp of the velocity information
 *
 * @fn void  VelocityModel::setTime(timeval t)
 * Set current time.
 * @param t time
 *
 * @fn void  VelocityModel::setTimeNow()
 * Get current time from system.
 *
 * @fn void  VelocityModel::getTime(long int *sec, long int *usec)
 * Get time from velocity.
 * @param sec contains seconds since the epoch upon return (Unix timestamp)
 * @param usec contains microseconds upon return
 *
 * @fn void  VelocityModel::getVelocity(float *vel_x, float *vel_y)
 * Method to retrieve velocity information
 * @param vel_x If not NULL contains velocity in X direction after call
 * @param vel_y If not NULL contains velocity in Y direction after call
 *
 * @fn float VelocityModel::getVelocityX()
 * Get velocity of tracked object in X direction.
 * @return velocity in m/s.
 *
 * @fn float VelocityModel::getVelocityY()
 * Get velocity of tracked object in X direction.
 * @return velocity in m/s.
 *
 * @fn void VelocityModel::calc()
 * Calculate velocity values from given data
 * This method must be called after all relevent data (set*) has been
 * set. After calc() the velocity values can be retrieved
 *
 * @fn void VelocityModel::reset()
 * Reset velocity model
 * Must be called if ball is not visible at any time
 *
 * @fn coordsys_type_t VelocityModel::getCoordinateSystem()
 * Returns the used coordinate system, must be either COORDSYS_ROBOT_CART or
 * COORDSYS_ROBOT_WORLD. ROBOT denotes velocities relative to the robot
 * (which can be tramsformed to global velocities by:
 * glob_vel_x = rel_vel_x * cos( robot_ori ) - rel_vel_y * sin( robot_ori )
 * WORLD denotes velocities in the robot coordinate system
 * glob_vel_y = rel_vel_x * sin( robot_ori ) + rel_vel_y * cos( robot_ori )
 */

/** Virtual empty destructor. */
VelocityModel::~VelocityModel()
{
}

} // end namespace firevision
