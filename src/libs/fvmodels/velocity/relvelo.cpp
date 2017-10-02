
/***************************************************************************
 *  relvelo.cpp - Implementation of velocity model based on relative ball
 *                 positions and relative robot velocity
 *
 *  Created: Tue Oct 04 15:54:27 2005
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

#include <fvmodels/velocity/relvelo.h>

#include <utils/system/console_colors.h>
#include <utils/time/time.h>

#include <limits>

#include <cmath>
#include <cstdlib>

using namespace std;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class VelocityFromRelative <fvmodels/velocity/relvelo.h>
 * Calculate velocity from relative positions.
 */

/** Constructor.
 * @param model relative position model
 * @param max_history_length maximum history length
 * @param calc_interval calculation interval
 */
VelocityFromRelative::VelocityFromRelative(RelativePositionModel* model,
					   unsigned int max_history_length,
					   unsigned int calc_interval)
{
  this->relative_pos_model = model;
  this->max_history_length = max_history_length;
  this->calc_interval      = calc_interval;

  //kalman_enabled = true;

  robot_rel_vel_x = robot_rel_vel_y = 0.f;

  velocity_x = velocity_y = 0.f;

  /*
  // initial variance for ball pos
  var_proc_x = 300;
  var_proc_y =  50;
  var_meas_x = 300;
  var_meas_y =  50;

  // initial variance for ball pos
  kalman_filter = new kalmanFilter2Dim();
  CMatrix<float> initialStateVarianceBall(2,2);
  initialStateVarianceBall[0][0] = var_meas_x;
  initialStateVarianceBall[1][0] =    0.0;
  initialStateVarianceBall[0][1] =    0.0;
  initialStateVarianceBall[1][1] = var_meas_y;
  kalman_filter->setInitialStateCovariance( initialStateVarianceBall ); 

  // process noise for ball pos kf, initial estimates, refined in calc()
  kalman_filter->setProcessCovariance( var_proc_x, var_proc_y );
  kalman_filter->setMeasurementCovariance( var_meas_x, var_meas_y );
  */

  avg_vx_sum = 0.f;
  avg_vx_num = 0;

  avg_vy_sum = 0.f;
  avg_vy_num = 0;

  ball_history.clear();

}


/** Destructor. */
VelocityFromRelative::~VelocityFromRelative()
{
}


void
VelocityFromRelative::setPanTilt(float pan, float tilt)
{
}


void
VelocityFromRelative::setRobotPosition(float x, float y, float ori, timeval t)
{
}


void
VelocityFromRelative::setRobotVelocity(float rel_vel_x, float rel_vel_y, timeval t)
{
  robot_rel_vel_x = rel_vel_x;
  robot_rel_vel_y = rel_vel_y;
  robot_rel_vel_t.tv_sec = t.tv_sec;
  robot_rel_vel_t.tv_usec = t.tv_usec;
}

void
VelocityFromRelative::setTime(timeval t)
{
  now.tv_sec  = t.tv_sec;
  now.tv_usec = t.tv_usec;
}


void
VelocityFromRelative::setTimeNow()
{
  gettimeofday(&now, NULL);
}


void
VelocityFromRelative::getTime(long int *sec, long int *usec)
{
  *sec  = now.tv_sec;
  *usec = now.tv_usec;
}


void
VelocityFromRelative::getVelocity(float *vel_x, float *vel_y)
{
  if (vel_x != NULL) {
    *vel_x = velocity_x;
  }
  if (vel_y != NULL) {
    *vel_y = velocity_y;
  }
}


float
VelocityFromRelative::getVelocityX()
{
  return velocity_x;
}


float
VelocityFromRelative::getVelocityY()
{
  return velocity_y;
}


void
VelocityFromRelative::calc()
{
  /*
  char user_input = toupper( getkey() );

  if ( ! relative_pos_model->isPosValid() ) {
    return;
  }

  if (user_input == 'P') {
    cout << "Enter new kalman process variance values (X Y):" << flush;
    cin >> var_proc_x >> var_proc_y;
  } else if (user_input == 'M') {
    cout << "Enter new kalman measurement variance values (X Y):" << flush;
    cin >> var_meas_x >> var_meas_y;
  } else if (user_input == 'R') {
    cout << "Reset" << endl;
    reset();
  } else if (user_input == 'C') {
    cout << "Current kalman measurement variance (X Y) = ("
	 << var_meas_x << " " << var_meas_y << ")" << endl
	 << "Current kalman process variance (X Y)     = ("
	 << var_proc_x << " " << var_proc_y << ")" << endl;
  } else if (user_input == 'K') {
    kalman_enabled = ! kalman_enabled;
    if ( kalman_enabled ) {
      cout << "Kalman filtering enabled" << endl;
      kalman_filter->reset();
    } else {
      cout << "Kalman filtering disabled" << endl;
    }
  }
  */

  // Gather needed data
  cur_ball_x = relative_pos_model->get_x();
  cur_ball_y = relative_pos_model->get_y();
  cur_ball_dist = relative_pos_model->get_distance();

  if ( isnan(cur_ball_x) || isinf(cur_ball_x) ||
       isnan(cur_ball_y) || isinf(cur_ball_y) ||
       isnan(cur_ball_dist) || isinf(cur_ball_dist) ) {
    // cout << cred << "relative position model returned nan/inf value(s)!" << cnormal << endl;
    return;
  }

  // if we project the last ball position by the velocity we calculated
  // at that time we can compare this to the current position and estimate
  // an error from this information
  if (last_available) {
    proj_time_diff_sec = fawkes::time_diff_sec(now, last_time);
    proj_x = last_x + velocity_x * proj_time_diff_sec;
    proj_y = last_y + velocity_y * proj_time_diff_sec;
    last_proj_error_x = cur_ball_x - proj_x;
    last_proj_error_y = cur_ball_y - proj_y;
    last_available = false;
  } else {
    last_proj_error_x = cur_ball_x;
    last_proj_error_y = cur_ball_x;
  }


  // newest entry first
  vel_postime_t *vpt = (vel_postime_t *)malloc(sizeof(vel_postime_t));;
  vpt->x             = cur_ball_x;
  vpt->y             = cur_ball_y;
  vpt->t.tv_sec      = now.tv_sec;
  vpt->t.tv_usec     = now.tv_usec;
  ball_history.push_front( vpt );
    

  if (ball_history.size() >= 2) {

    // we need at least two entries
    // take the last robot velocity, then find the corresponding ball_pos entry
    // in the history and an entry about 100ms away to extrapolate the
    // ball velocity, then correct this by the robot's velocity we got

    if ( fawkes::time_diff_sec(robot_rel_vel_t, vel_last_time) != 0 ) {
      // We have a new robot position data, calculate new velocity

      vel_last_time.tv_sec  = robot_rel_vel_t.tv_sec;
      vel_last_time.tv_usec = robot_rel_vel_t.tv_usec;

      f_diff_sec = numeric_limits<float>::max();
      float time_diff;
      vel_postime_t *young = NULL;
      vel_postime_t *old   = NULL;
      unsigned int  step = 0;
      for (bh_it = ball_history.begin(); bh_it != ball_history.end(); ++bh_it) {
	// Find the ball pos history entry closest in time (but still younger) to
	// the new position data
	time_diff = fawkes::time_diff_sec((*bh_it)->t, robot_rel_vel_t);
	if ( (time_diff > 0) && (time_diff < f_diff_sec) ) {
	  f_diff_sec = time_diff;
	  young = (*bh_it);
	} else {
	  // Now find second time
	  if (step != calc_interval) {
	    ++step;
	  } else {
	    // Found a second time
	    old = *bh_it;
	    ++bh_it;
	    break;
	  }
	}
      }

      if ((young != NULL) && (old != NULL)) {
	// we found two valid times

	diff_x = young->x - old->x;
	diff_y = young->y - old->y;
      
	f_diff_sec = fawkes::time_diff_sec(young->t, old->t);

	velocity_x = diff_x / f_diff_sec;
	velocity_y = diff_y / f_diff_sec;

	//cout << "f_diff_sec=" << f_diff_sec << "  vx=" << velocity_x << "  vy=" << velocity_y << endl;
	
	velocity_x += robot_rel_vel_x;
	velocity_y += robot_rel_vel_y;

	velocity_x -= last_proj_error_x * proj_time_diff_sec;
	velocity_y -= last_proj_error_y * proj_time_diff_sec;

	//cout << "vx+rx=" << velocity_x << "  vy+ry=" << velocity_y << endl;
	
	/*
	cout << endl
	     << "VELOCITY CALCULATION" << endl
	     << "  History size  : " << ball_history.size() << endl
	     << "  Ball position" << endl
	     << "    young       : (" << young->x << ", " << young->y << ")" << endl
	     << "    old         : (" << old->x << ", " << old->y << ")" << endl
	     << "    difference  : " << diff_x << ", " << diff_y << ")" << endl
	     << "  Time" << endl
	     << "    current     :  " << young->t.tv_sec << " sec, " << young->t.tv_usec << " usec" << endl
	     << "    old         :  " << old->t.tv_sec << " sec, " << old->t.tv_usec << " usec" << endl
	     << "    difference  :  " << f_diff_sec << " sec" << endl
	     << "  Projection" << endl
	     << "  proj error    : (" << last_proj_error_x << "," << last_proj_error_y << ")" << endl
	     << "  Velocity" << endl
	     << "    robot       : (" << robot_rel_vel_x << ", " << robot_rel_vel_y << ")" << endl
	     << "    Ball" << endl
	     << "      raw       : (" << velocity_x - robot_rel_vel_x << ", " << velocity_y - robot_rel_vel_y << ")" << endl
	     << "      corrected : (" << velocity_x << ", " << velocity_y << ")" << endl;
	*/

	/*
	if ( kalman_enabled ) {
	  applyKalmanFilter();
	}
	*/

	last_x = cur_ball_x;
	last_y = cur_ball_y;
	last_time.tv_sec = now.tv_sec;
	last_time.tv_usec = now.tv_usec;
	last_available = true;

	/*
	cout << "      filtered  : (" << clightpurple << velocity_x << cnormal
	     << ", " << clightpurple << velocity_y << cnormal << ")" << endl
	     << endl;
	*/

	// erase old history entries
	if (bh_it != ball_history.end()) {
	  ball_history.erase(bh_it, ball_history.end());
	}
      } else {
	// cout << "did not find matching young and old record" << endl;
	velocity_x = 0.f;
	velocity_y = 0.f;
      }
    } else {
      // we did not get a new robot position, keep old velocities for 2 seconds
      if (fawkes::time_diff_sec(now, vel_last_time) > 2) {
	// cout << "did not get new robot position for more than 2 sec, resetting" << endl;
	velocity_x = 0.f;
	velocity_y = 0.f;
      }
    }
  } else {
    // cout << "history too short" << endl;
    velocity_x = 0.f;
    velocity_y = 0.f;
  }

  if (ball_history.size() > max_history_length) {
    bh_it = ball_history.begin();
    for (unsigned int i = 0; i < max_history_length; ++i) {
      ++bh_it;
    }
    ball_history.erase(bh_it, ball_history.end());
  }

}


void
VelocityFromRelative::reset()
{
  /*
  if (kalman_enabled) {
    kalman_filter->reset();
  }
  */
  avg_vx_sum = 0.f;
  avg_vx_num = 0;
  avg_vy_sum = 0.f;
  avg_vy_num = 0;
  velocity_x = 0.f;
  velocity_y = 0.f;
  ball_history.clear();
}


const char *
VelocityFromRelative::getName() const
{
  return "VelocityModel::VelocityFromRelative";
}


coordsys_type_t
VelocityFromRelative::getCoordinateSystem()
{
  return COORDSYS_ROBOT_CART;
}

/*
void
VelocityFromRelative::applyKalmanFilter()
{
  /
  avg_vx_sum += velocity_x;
  avg_vy_sum += velocity_y;

  ++avg_vx_num;
  ++avg_vy_num;

  avg_vx = avg_vx_sum / avg_vx_num;
  avg_vy = avg_vy_sum / avg_vy_num;

  age_factor = (fawkes::time_diff_sec(now, robot_rel_vel_t) + f_diff_sec);

  rx = (velocity_x - avg_vx) * age_factor;
  ry = (velocity_y - avg_vy) * age_factor;

  kalman_filter->setProcessCovariance( rx * rx, ry * ry );

  rx = (velocity_x - avg_vx) * cur_ball_dist;
  ry = (velocity_y - avg_vy) * cur_ball_dist;

  kalman_filter->setMeasurementCovariance( rx * rx, ry * ry );
  /

  kalman_filter->setProcessCovariance( var_proc_x * cur_ball_dist,
				       var_proc_y * cur_ball_dist);
  kalman_filter->setMeasurementCovariance( var_meas_x * cur_ball_dist,
					   var_meas_y * cur_ball_dist );

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
