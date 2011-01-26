
/***************************************************************************
 *  boxrelative.cpp - Implementation of the relative box position model
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
#include <fvmodels/relative_position/box_relative.h>
#include <utils/math/angle.h>

#include <iostream>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BoxRelative <fvmodels/relative_position/box_relative.h>
 * Relative (beer) box position model.
 * Model used in Bremen to get world champions :-)
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
 */
BoxRelative::BoxRelative(unsigned int image_width,
			 unsigned int image_height,
			 float camera_height,
			 float camera_offset_x, float camera_offset_y,
			 float camera_ori,
			 float horizontal_angle, float vertical_angle
			 )
{

  this->image_width        = image_width;
  this->image_height       = image_height;
  this->horizontal_angle   = deg2rad( horizontal_angle );
  this->vertical_angle     = deg2rad( vertical_angle   );
  this->camera_orientation = deg2rad( camera_ori       );
  this->camera_height      = camera_height;
  this->camera_offset_x    = camera_offset_x;
  this->camera_offset_y    = camera_offset_y;

  center.x = center.y = 0.f;
  pan	         = 0.0f;
  tilt           = 0.0f;

  pan_rad_per_pixel  = this->horizontal_angle   / this->image_width;
  tilt_rad_per_pixel = this->vertical_angle     / this->image_height;

  box_x = box_y = bearing = slope = distance_box_motor = distance_box_cam = 0.f;

  DEFAULT_X_VARIANCE = 1500.f;
  DEFAULT_Y_VARIANCE = 1000.f;

  /*
  var_proc_x = 1500;
  var_proc_y = 1000;
  var_meas_x = 1500;
  var_meas_y = 1000;

  // initial variance for box pos kf
  kalman_filter = new kalmanFilter2Dim();
  CMatrix<float> initialStateVarianceBox(2,2);
  initialStateVarianceBox[0][0] = DEFAULT_X_VARIANCE;
  initialStateVarianceBox[1][0] = 0.00;
  initialStateVarianceBox[0][1] = 0.00;
  initialStateVarianceBox[1][1] = DEFAULT_Y_VARIANCE;
  kalman_filter->setInitialStateCovariance( initialStateVarianceBox ); 

  // process noise for box pos kf
  kalman_filter->setProcessCovariance( DEFAULT_X_VARIANCE, DEFAULT_Y_VARIANCE );
  kalman_filter->setMeasurementCovariance( DEFAULT_X_VARIANCE, DEFAULT_Y_VARIANCE );
  */
}


/* Get the distance to the box - NOT IMPLEMENTED!
 * Was not needed, matching with laser data.
 * @return 0
 */
float
BoxRelative::get_distance() const 
{
  return distance_box_motor;
}


float
BoxRelative::get_bearing(void) const
{
  return bearing;
}


float
BoxRelative::get_slope() const
{
  return slope;
}


/* Get relative Y distance in local cartesian coordinate system - NOT IMPLEMENTED!
 * Was not needed, matching with laser data.
 * @return 0
 */
float
BoxRelative::get_y(void) const
{
  return box_y;
}


/* Get relative X distance in local cartesian coordinate system - NOT IMPLEMENTED!
 * Was not needed, matching with laser data.
 * @return 0
 */
float
BoxRelative::get_x(void) const
{
  return box_x;
}

void
BoxRelative::set_radius(float r)
{
}


void
BoxRelative::set_center(float x, float y)
{
  center.x = x;
  center.y = y;
}


void
BoxRelative::set_center(const center_in_roi_t& c)
{
  center.x = c.x;
  center.y = c.y;
}


void
BoxRelative::set_pan_tilt(float pan, float tilt)
{
  this->pan  = pan;
  this->tilt = tilt;
}


void
BoxRelative::get_pan_tilt(float *pan, float *tilt) const
{
  *pan  = this->pan;
  *tilt = this->tilt;
}


const char *
BoxRelative::get_name() const
{
  return "BoxRelative";
}


/** Set the horizontal viewing angle.
 * @param angle_deg horizontal viewing angle in degrees
 */
void
BoxRelative::set_horizontal_angle(float angle_deg)
{
  horizontal_angle = deg2rad( angle_deg );
}


/** Set the vertical viewing angle.
 * @param angle_deg vertical viewing angle in degrees
 */
void
BoxRelative::set_vertical_angle(float angle_deg)
{
  vertical_angle = deg2rad( angle_deg );
}


void
BoxRelative::reset()
{
  last_available = false;
  // kalman_filter->reset();
}

void
BoxRelative::calc()
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


  calc_unfiltered();
  // applyKalmanFilter();

}


bool
BoxRelative::is_pos_valid() const
{
  return true;
}


void
BoxRelative::calc_unfiltered()
{
  /* Pan to the right is positive. Therefore we add it,
     because bearing to the right shall be positive */
  bearing = ((center.x - image_width/2) * pan_rad_per_pixel + pan + camera_orientation);

  // invert sign, because slope downward shall be negative
  slope = -((center.y - image_height / 2) * tilt_rad_per_pixel - tilt);

  distance_box_cam = camera_height * tan(M_PI / 2 + slope);
  distance_box_motor = distance_box_cam - camera_offset_x;

  /*
  cout << "pan:" << pan << "  tilt:" << tilt
       << "  bearing: " << bearing << "  slope:" << slope
       << "  dist->cam:" << distance_box_cam
       << "  dist->motor:" << distance_box_motor
       << endl;
  */

  box_x = cos( bearing ) * distance_box_cam + camera_offset_x;
  box_y = sin( bearing ) * distance_box_cam + camera_offset_y;
}


/*
void
BoxRelative::applyKalmanFilter()
{

  kalman_filter->setMeasurementCovariance( var_meas_x, var_meas_y );
  kalman_filter->setProcessCovariance( var_proc_x, var_proc_y );

  last_x = box_x;
  last_y = box_y;

  kalman_filter->setMeasurementX( box_x );
  kalman_filter->setMeasurementY( box_y );
  kalman_filter->doCalculation();

  box_x = kalman_filter->getStateX();
  box_y = kalman_filter->getStateY();

  if ( isnan( box_x ) || isinf( box_x ) ||
       isnan( box_y ) || isinf( box_y ) ) {
    // Kalman is wedged, use unfiltered result and reset filter
    kalman_filter->reset();
    box_x = last_x;
    box_y = last_y;
  }

}
*/

} // end namespace firevision
