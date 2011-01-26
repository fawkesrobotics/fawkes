
/***************************************************************************
 *  front_ball.cpp - Implementation of the relative ball position model for
 *                   the front vision
 *
 *  Created: Fri Jun 03 22:56:22 2005
 *  Copyright  2005       Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
 *             2005-2006  Tim Niemueller [www.niemueller.de]
 *             2005       Martin Heracles <Martin.Heracles@rwth-aachen.de>
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
#include <iostream>
#include <fvmodels/relative_position/front_ball.h>
#include <utils/math/angle.h>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FrontBallRelativePos <fvmodels/relative_position/front_ball.h>
 * Relative ball position model for front vision.
 */

/** Constructor.
 * @param image_width width of image in pixels
 * @param image_height height of image in pixels
 * @param camera_height height of camera in meters
 * @param camera_offset_x camera offset of the motor axis in x direction
 * @param camera_offset_y camera offset of the motor axis in y direction
 * @param camera_ori camera orientation compared to the robot
 * @param horizontal_angle horizontal viewing angle (in degree)
 * @param vertical_angle vertical viewing angle (in degree)
 * @param ball_circumference ball circumference
 */
FrontBallRelativePos::FrontBallRelativePos(unsigned int image_width,
					   unsigned int image_height,
					   float camera_height,
					   float camera_offset_x,
					   float camera_offset_y,
					   float camera_ori,
					   float horizontal_angle,
					   float vertical_angle,
					   float ball_circumference
					   )
{

  this->image_width        = image_width;
  this->image_height       = image_height;
  this->ball_circumference = ball_circumference;
  this->horizontal_angle   = deg2rad( horizontal_angle );
  this->vertical_angle     = deg2rad( vertical_angle   );
  this->camera_orientation = deg2rad( camera_ori       );
  this->camera_height      = camera_height;
  this->camera_offset_x    = camera_offset_x;
  this->camera_offset_y    = camera_offset_y;

  m_fRadius         = 0.0f;
  m_cirtCenter.x    = 0.0f;
  m_cirtCenter.y    = 0.0f;
  m_fPan	    = 0.0f;
  m_fTilt	    = 0.0f;

  avg_x = avg_y = avg_x_sum = avg_y_sum = 0.f;
  avg_x_num = avg_y_num = 0;

  m_fPanRadPerPixel  = this->horizontal_angle   / this->image_width;
  m_fTiltRadPerPixel = this->vertical_angle     / this->image_height;
  m_fBallRadius      = this->ball_circumference / ( 2 * M_PI );

  ball_x = ball_y = bearing = slope = distance_ball_motor = distance_ball_cam = 0.f;

  DEFAULT_X_VARIANCE = 1500.f;
  DEFAULT_Y_VARIANCE = 1000.f;

  var_proc_x = 1500;
  var_proc_y = 1000;
  var_meas_x = 1500;
  var_meas_y = 1000;

  /*
  // initial variance for ball pos kf
  kalman_filter = new kalmanFilter2Dim();
  CMatrix<float> initialStateVarianceBall(2,2);
  initialStateVarianceBall[0][0] = DEFAULT_X_VARIANCE;
  initialStateVarianceBall[1][0] = 0.00;
  initialStateVarianceBall[0][1] = 0.00;
  initialStateVarianceBall[1][1] = DEFAULT_Y_VARIANCE;
  kalman_filter->setInitialStateCovariance( initialStateVarianceBall ); 

  // process noise for ball pos kf
  kalman_filter->setProcessCovariance( DEFAULT_X_VARIANCE, DEFAULT_Y_VARIANCE );
  kalman_filter->setMeasurementCovariance( DEFAULT_X_VARIANCE, DEFAULT_Y_VARIANCE );
  */
}


float
FrontBallRelativePos::get_distance() const 
{
  return distance_ball_motor;
}


float
FrontBallRelativePos::get_bearing(void) const
{
  return bearing;
}


float
FrontBallRelativePos::get_slope() const
{
  return slope;
}


float
FrontBallRelativePos::get_y(void) const
{
  return ball_y;
}


float
FrontBallRelativePos::get_x(void) const
{
  return ball_x;
}


void
FrontBallRelativePos::set_center(float x, float y)
{
  m_cirtCenter.x = x;
  m_cirtCenter.y = y;
}


void
FrontBallRelativePos::set_center(const center_in_roi_t& c)
{
  m_cirtCenter.x = c.x;
  m_cirtCenter.y = c.y;
}


void
FrontBallRelativePos::set_radius(float r)
{
  m_fRadius = r;
}


/** Get the ball radius.
 * @return ball radius
 */
float
FrontBallRelativePos::get_radius() const
{
  return m_fRadius;
}


void
FrontBallRelativePos::set_pan_tilt(float pan, float tilt)
{
  m_fPan = pan;
  m_fTilt = tilt;
}


void
FrontBallRelativePos::get_pan_tilt(float *pan, float *tilt) const
{
  *pan  = m_fPan;
  *tilt = m_fTilt;
}


const char *
FrontBallRelativePos::get_name() const
{
  return "FrontBallRelativePos";
}


/** Set horizontal viewing angle.
 * @param angle_deg horizontal viewing angle in degree
 */
void
FrontBallRelativePos::set_horizontal_angle(float angle_deg)
{
  horizontal_angle = deg2rad( angle_deg );
}


/** Set vertical viewing angle.
 * @param angle_deg horizontal viewing angle in degree
 */
void
FrontBallRelativePos::set_vertical_angle(float angle_deg)
{
  vertical_angle = deg2rad( angle_deg );
}


void
FrontBallRelativePos::reset()
{
  last_available = false;
  //kalman_filter->reset();
}

void
FrontBallRelativePos::calc()
{

  /*
  char user_input = toupper( getkey() );

  if (user_input == 'P') {
    cout << "Enter new kalman process variance values (X Y):" << flush;
    cin >> var_proc_x >> var_proc_y;
  } else if (user_input == 'M') {
    cout << "Enter new kalman measurement variance values (X Y):" << flush;
    cin >> var_meas_x >> var_meas_y;
  } else if (user_input == 'R') {
    cout << "Reset" << endl;
    reset();
  }
  */

  float tmp = m_fBallRadius / sin(m_fRadius * m_fPanRadPerPixel);
  
  /* projection of air-line-distance on the ground (Pythagoras) */
  distance_ball_cam = sqrt( tmp * tmp    -
	                    (camera_height - m_fBallRadius) * (camera_height - m_fBallRadius)  );


#ifdef OLD_COORD_SYS
  /* Bearing shall be clockwise positive. */
  bearing =   (((m_cirtCenter.x - image_width/2) * m_fPanRadPerPixel + m_fPan + camera_orientation));
#else
  /* Bearing shall be counter-clockwise positive. */
  bearing = - (((m_cirtCenter.x - image_width/2) * m_fPanRadPerPixel + m_fPan + camera_orientation));
#endif

  /* Slope shall be downward negative */
  slope   = - ((m_cirtCenter.y - image_height / 2) * m_fTiltRadPerPixel - m_fTilt);

  ball_x = cos( bearing ) * distance_ball_cam + camera_offset_x;
  ball_y = sin( bearing ) * distance_ball_cam + camera_offset_y;

  // applyKalmanFilter();

  distance_ball_motor = sqrt( ball_x * ball_x  +  ball_y * ball_y );

}


bool
FrontBallRelativePos::is_pos_valid() const
{
  return true;
}


void
FrontBallRelativePos::calc_unfiltered()
{
  float tmp = m_fBallRadius / sin(m_fRadius * m_fPanRadPerPixel);
  
  /* projection of air-line-distance on the ground
     (Pythagoras) */
  distance_ball_cam = sqrt( tmp                             * tmp                            -
	                    (camera_height - m_fBallRadius) * (camera_height - m_fBallRadius)  );


#ifdef OLD_COORD_SYS
  /* Bearing shall be clockwise positive. */
  bearing =   (((m_cirtCenter.x - image_width/2) * m_fPanRadPerPixel + m_fPan + camera_orientation));
#else
  /* Bearing shall be counter-clockwise positive. */
  bearing = - (((m_cirtCenter.x - image_width/2) * m_fPanRadPerPixel + m_fPan + camera_orientation));
#endif

  // invert sign, because slope downward shall be negative
  slope   = - ((m_cirtCenter.y - image_height / 2) * m_fTiltRadPerPixel - m_fTilt);


  ball_x = cos( bearing ) * distance_ball_cam + camera_offset_x;
  ball_y = sin( bearing ) * distance_ball_cam + camera_offset_y;

  distance_ball_motor = sqrt( ball_x * ball_x  +  ball_y * ball_y );

}


/*
void
FrontBallRelativePos::applyKalmanFilter()
{

  kalman_filter->setMeasurementCovariance( var_meas_x, var_meas_y );
  kalman_filter->setProcessCovariance( var_proc_x, var_proc_y );

  last_x = ball_x;
  last_y = ball_y;

  kalman_filter->setMeasurementX( ball_x );
  kalman_filter->setMeasurementY( ball_y );
  kalman_filter->doCalculation();

  ball_x = kalman_filter->getStateX();
  ball_y = kalman_filter->getStateY();

  if ( isnan( ball_x ) || isinf( ball_x ) ||
       isnan( ball_y ) || isinf( ball_y ) ) {
    // Kalman is wedged, use unfiltered result and reset filter
    kalman_filter->reset();
    ball_x = last_x;
    ball_y = last_y;
  }

}
*/

} // end namespace firevision
