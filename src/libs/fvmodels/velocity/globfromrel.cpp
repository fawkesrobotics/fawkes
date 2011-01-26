
/***************************************************************************
 *  globfromrel.cpp - Implementation of velocity model based on relative
 *                    ball positions and relative robot velocity
 *
 *  Created: Fri Oct 21 11:19:03 2005
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

#include <core/exception.h>
#include <cmath>
#include <fvmodels/velocity/globfromrel.h>
#include <utils/time/time.h>

// #include "utils/system/console_colors.h"
// #include "utils/system/time.h"

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class VelocityGlobalFromRelative <fvmodels/velocity/globfromrel.h>
 * Global velocity from relative velocities.
 */

/** Destructor.
 * @param rel_velo_model relative velocity model
 * @param rel_pos_model relative position model
 */
VelocityGlobalFromRelative::VelocityGlobalFromRelative(VelocityModel *rel_velo_model,
						       RelativePositionModel *rel_pos_model)
{
  this->relative_velocity = rel_velo_model;
  this->relative_position = rel_pos_model;

  if ( rel_velo_model->getCoordinateSystem() != COORDSYS_ROBOT_CART ) {
    /*
    cout << cblue << "VelocityGlobalFromRelative::Constructor: " << cred
	 << "Given velocity model does not return robot-centric cartesian coordinates. WILL NOT WORK!"
	 << cnormal << endl;
    */
    throw Exception("Given velocity model does not return robot-centric cartesian coordinates. WILL NOT WORK!");
  }

  robot_ori = robot_poseage = 0.f;

  velocity_x = velocity_y = 0.f;

  /*
  // initial variance for ball pos kf
  kalman_filter = new kalmanFilter2Dim();
  CMatrix<float> initialStateVarianceBall(2,2);
  initialStateVarianceBall[0][0] = 5.00;
  initialStateVarianceBall[1][0] = 0.00;
  initialStateVarianceBall[0][1] = 0.00;
  initialStateVarianceBall[1][1] = 5.00;
  kalman_filter->setInitialStateCovariance( initialStateVarianceBall ); 

  // process noise for ball pos kf, initial estimates, refined in calc()
  kalman_filter->setProcessCovariance( 1.f, 1.f );
  kalman_filter->setMeasurementCovariance( 4.f, 4.f );

  avg_vx_sum = 0.f;
  avg_vx_num = 0;

  avg_vy_sum = 0.f;
  avg_vy_num = 0;
  */
}


/** Destructor. */
VelocityGlobalFromRelative::~VelocityGlobalFromRelative()
{
}


void
VelocityGlobalFromRelative::setPanTilt(float pan, float tilt)
{
}


void
VelocityGlobalFromRelative::setRobotPosition(float x, float y, float ori, timeval t)
{
  timeval now;
  gettimeofday(&now, 0);
  robot_ori     = ori;
  robot_poseage = time_diff_sec(now, t);
}


void
VelocityGlobalFromRelative::setRobotVelocity(float rel_vel_x, float rel_vel_y, timeval t)
{
}

void
VelocityGlobalFromRelative::setTime(timeval t)
{
}


void
VelocityGlobalFromRelative::setTimeNow()
{
}


void
VelocityGlobalFromRelative::getTime(long int *sec, long int *usec)
{
  *sec  = 0;
  *usec = 0;
}


void
VelocityGlobalFromRelative::getVelocity(float *vel_x, float *vel_y)
{
  if (vel_x != 0) {
    *vel_x = velocity_x;
  }
  if (vel_y != 0) {
    *vel_y = velocity_y;
  }
}


float
VelocityGlobalFromRelative::getVelocityX()
{
  return velocity_x;
}


float
VelocityGlobalFromRelative::getVelocityY()
{
  return velocity_y;
}



void
VelocityGlobalFromRelative::calc()
{
  
  relative_velocity->getVelocity( &rel_vel_x, &rel_vel_y );
  sin_ori   = sin( robot_ori );
  cos_ori   = cos( robot_ori );
  rel_dist  = relative_position->get_distance();

  velocity_x = rel_vel_x * cos_ori - rel_vel_y * sin_ori;
  velocity_y = rel_vel_x * sin_ori + rel_vel_y * cos_ori;
    
  // applyKalmanFilter();

}


void
VelocityGlobalFromRelative::reset()
{
  // kalman_filter->reset();
  avg_vx_sum = 0.f;
  avg_vx_num = 0;
  avg_vy_sum = 0.f;
  avg_vy_num = 0;
  velocity_x = 0.f;
  velocity_y = 0.f;
}


const char *
VelocityGlobalFromRelative::getName() const
{
  return "VelocityModel::VelocityGlobalFromRelative";
}


coordsys_type_t
VelocityGlobalFromRelative::getCoordinateSystem()
{
  return COORDSYS_WORLD_CART;
}


/*
void
VelocityGlobalFromRelative::applyKalmanFilter()
{
  avg_vx_sum += velocity_x;
  avg_vy_sum += velocity_y;

  ++avg_vx_num;
  ++avg_vy_num;

  avg_vx = avg_vx_sum / avg_vx_num;
  avg_vy = avg_vy_sum / avg_vy_num;

  rx = (velocity_x - avg_vx) * robot_poseage;
  ry = (velocity_y - avg_vy) * robot_poseage;

  kalman_filter->setProcessCovariance( rx * rx, ry * ry );

  rx = (velocity_x - avg_vx) * rel_dist;
  ry = (velocity_y - avg_vy) * rel_dist;

  kalman_filter->setMeasurementCovariance( rx * rx, ry * ry );

  kalman_filter->setMeasurementX( velocity_x );
  kalman_filter->setMeasurementY( velocity_y );
  kalman_filter->doCalculation();

  velocity_x = kalman_filter->getStateX();
  velocity_y = kalman_filter->getStateY();

  velocity_x = round( velocity_x * 10 ) / 10;
  velocity_y = round( velocity_y * 10 ) / 10;

  if (isnan(velocity_x) || isinf(velocity_x) ||
      isnan(velocity_y) || isinf(velocity_y) ) {
    reset();
  }

}
*/

} // end namespace firevision
