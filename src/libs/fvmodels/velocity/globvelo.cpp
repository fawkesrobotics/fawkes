
/***************************************************************************
 *  globvelo.cpp - Implementation of velocity model based on global
 *                 positions
 *
 *  Created: Mon Sep 05 17:12:48 2005
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

#include <cmath>
#include <fvmodels/velocity/globvelo.h>

#include <utils/time/time.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class VelocityFromGlobal <fvmodels/velocity/globvelo.h>
 * Velocity from global positions.
 */

/** Constructor.
 * @param model global position model
 * @param history_length maximum history length
 * @param calc_interval calculation interval
 */
VelocityFromGlobal::VelocityFromGlobal(GlobalPositionModel* model,
				       unsigned int history_length,
				       unsigned int calc_interval)
{
  this->global_pos_model = model;
  this->history_length   = history_length;
  this->calc_interval    = calc_interval;

  robot_pos_x = robot_pos_y = robot_pos_ori = 0.0f;

  velocity_x = velocity_y = 0.f;

  /*
  // initial variance for ball pos kf
  kalman_filter = new kalmanFilter2Dim();
  CMatrix<float> initialStateVarianceBall(2,2);
  initialStateVarianceBall[0][0] = 2.00;
  initialStateVarianceBall[1][0] = 0.00;
  initialStateVarianceBall[0][1] = 0.00;
  initialStateVarianceBall[1][1] = 2.00;
  kalman_filter->setInitialStateCovariance( initialStateVarianceBall ); 

  // process noise for ball pos kf
  CMatrix<float> processVarianceBall(2,2);
  processVarianceBall[0][0] = 0.50;
  processVarianceBall[1][0] = 0.00;
  processVarianceBall[0][1] = 0.00;
  processVarianceBall[1][1] = 0.50;
  kalman_filter->setProcessCovariance( processVarianceBall );

  kalman_filter->setMeasurementCovariance( 0.5, 0.5 );
  */
}


/** Destructor. */
VelocityFromGlobal::~VelocityFromGlobal()
{
}


void
VelocityFromGlobal::setPanTilt(float pan, float tilt)
{
}


void
VelocityFromGlobal::setRobotPosition(float x, float y, float ori, timeval t)
{
  timeval _now;
  gettimeofday(&_now, 0);
  robot_pos_x   = x;
  robot_pos_y   = y;
  robot_pos_ori = ori;
  robot_pos_age = fawkes::time_diff_sec(_now, t);
}


void
VelocityFromGlobal::setRobotVelocity(float vel_x, float vel_y, timeval t)
{
}


void
VelocityFromGlobal::setTime(timeval t)
{
  now.tv_sec  = t.tv_sec;
  now.tv_usec = t.tv_usec;
}


void
VelocityFromGlobal::setTimeNow()
{
  gettimeofday(&now, 0);
}


void
VelocityFromGlobal::getTime(long int *sec, long int *usec)
{
  *sec  = now.tv_sec;
  *usec = now.tv_usec;
}


void
VelocityFromGlobal::getVelocity(float *vel_x, float *vel_y)
{
  if (vel_x != 0) {
    *vel_x = velocity_x;
  }
  if (vel_y != 0) {
    *vel_y = velocity_y;
  }
}


float
VelocityFromGlobal::getVelocityX()
{
  return velocity_x;
}


float
VelocityFromGlobal::getVelocityY()
{
  return velocity_y;
}



void
VelocityFromGlobal::calc()
{

  // Gather needed data
  current_x = global_pos_model->get_x();
  current_y = global_pos_model->get_y();

  last_x.push_back( current_x );
  last_y.push_back( current_y );

  last_time.push_back(now);

  velocity_total_x = 0.f;
  velocity_total_y = 0.f;
  velocity_num     = 0;

  if (last_x.size() > calc_interval) {
    // min of sizes
    unsigned int m = (last_x.size() < last_y.size()) ? last_x.size() : last_y.size() ;
    for (unsigned int i = calc_interval; i < m; i += calc_interval) {
      diff_x = last_x[i] - last_x[i - calc_interval];
      diff_y = last_y[i] - last_y[i - calc_interval];

      diff_sec  = last_time[i].tv_sec  - last_time[i - calc_interval].tv_sec;
      diff_usec = last_time[i].tv_usec - last_time[i - calc_interval].tv_usec;
      if (diff_usec < 0) {
	diff_sec  -= 1;
	diff_usec += 1000000;
      }

      f_diff_sec = diff_sec + diff_usec / 1000000.f;
      
      velocity_total_x += diff_x / f_diff_sec;
      velocity_total_y += diff_y / f_diff_sec;
      velocity_num++;
    }
  }

  // Get average velocity
  velocity_x = velocity_total_x / velocity_num;
  velocity_y = velocity_total_y / velocity_num;

  // applyKalmanFilter();

  while (last_x.size() > history_length) {
    last_x.erase( last_x.begin() );
    last_y.erase( last_y.begin() );
  }

}


void
VelocityFromGlobal::reset()
{
  // kalman_filter->reset();
}


const char *
VelocityFromGlobal::getName() const
{
  return "VelocityModel::VelocityFromGlobal";
}


coordsys_type_t
VelocityFromGlobal::getCoordinateSystem()
{
  return COORDSYS_WORLD_CART;
}


/*
void
VelocityFromGlobal::applyKalmanFilter()
{
  kalman_filter->setMeasurementX( velocity_x );
  kalman_filter->setMeasurementY( velocity_y );
  kalman_filter->doCalculation();

  velocity_x = kalman_filter->getStateX();
  velocity_y = kalman_filter->getStateY();

  if (isnan(velocity_x) || isinf(velocity_x)) {
    kalman_filter->reset();
    velocity_x = 0.f;
  }

  if (isnan(velocity_y) || isinf(velocity_y)) {
    kalman_filter->reset();
    velocity_y = 0.f;
  }

}
*/

} // end namespace firevision
