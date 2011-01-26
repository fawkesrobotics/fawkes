
/***************************************************************************
 *  relativepositionmodel.cpp - Abstract class defining a position model for
 *                            calculation of relative position
 *
 *  Created: Wed Mar 21 15:54:42 2007
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

#include <fvmodels/relative_position/relativepositionmodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class RelativePositionModel <fvmodels/relative_position/relativepositionmodel.h>
 * Relative Position Model Interface.
 * This interfaces defines a relative position model.
 *
 * @fn const char * RelativePositionModel::get_name() const
 * Get name of relative position model.
 * @return name of relative position model
 *
 * @fn void RelativePositionModel::set_radius(float r)
 * Set radius of a found circle.
 * This is especially used for ball position implementations.
 * @param r radius
 *
 * @fn void RelativePositionModel::set_center(float x, float y)
 * Set center of a found circle.
 * This is especially used for ball position implementations.
 * @param x x position in image (pixels)
 * @param y y position in image (pixels)
 *
 * @fn void RelativePositionModel::set_center(const center_in_roi_t& c)
 * Set center of a found circle.
 * This is especially used for ball position implementations.
 * @param c center
 *
 * @fn void RelativePositionModel::set_pan_tilt(float pan, float tilt)
 * Set camera pan and tilt.
 * @param pan pan value (rad)
 * @param tilt tilt value (rad)
 *
 * @fn void RelativePositionModel::get_pan_tilt(float *pan, float *tilt) const
 * Get camera pan tilt.
 * @param pan contains pan value (rad) upon return
 * @param tilt contains tilt value (rad) upon return
 *
 * @fn void RelativePositionModel::calc()
 * Calculate position data.
 * Call this method if all relevant data (set(Radius|Center|PanTilt))
 * has been set, after this valid data can be retrieved via get*
 *
 * @fn void RelativePositionModel::calc_unfiltered()
 * Calculate data unfiltered.
 * Same as calc(), but without any filtering (i.e. no Kalman filter).
 *
 * @fn void RelativePositionModel::reset()
 * Reset all data.
 * This must be called if the object is not visible.
 *
 * @fn float RelativePositionModel::get_distance() const
 * Get distance to object.
 * @return distance to object in meters.
 *
 * @fn float RelativePositionModel::get_bearing() const
 * Get bearing (horizontal angle) to object.
 * @return bearing in rad
 *
 * @fn float RelativePositionModel::get_slope() const
 * Get slope (vertical angle) to object.
 * @return slope in rad
 *
 * @fn float RelativePositionModel::get_x() const
 * Get relative X coordinate of object.
 * @return relative X coordinate in local metric cartesian coordinate system
 *
 * @fn float RelativePositionModel::get_y() const
 * Get relative Y coordinate of object.
 * @return relative Y coordinate in local metric cartesian coordinate system
 *
 * @fn bool RelativePositionModel::is_pos_valid() const
 * Check if position is valid.
 * @return true, if the calculated position is valid, false otherwise
 *
 * @author Tim Niemueller
 */


/** Destructor. */
RelativePositionModel::~RelativePositionModel()
{
}

/** Sets the camera orientation
 * @param pan pan value (rad)
 * @param tilt tilt value (rad)
 * @param roll roll value (rad)
 */
void
RelativePositionModel::set_cam_rotation(float pan, float tilt, float roll)
{
}

/** Returns the camera orientation
 * @param pan pan value (rad)
 * @param tilt tilt value (rad)
 * @param roll roll value (rad)
 */
void
RelativePositionModel::get_cam_rotation(float &pan, float &tilt, float &roll) const
{
  roll = 0;
  get_pan_tilt(&pan, &tilt);
}

/** Sets the current translation of the camera
 * @param height height of the camera [m]
 * @param rel_x distance to the center of the robot [m]
 * @param rel_y distance to the center of the robot [m]
 */
void
RelativePositionModel::set_cam_translation(float height, float rel_x, float rel_y)
{
}

/** Returns the current translation of the camera
 * @param height height of the camera [m]
 * @param rel_x distance to the center of the robot [m]
 * @param rel_y distance to the center of the robot [m]
 */
void
RelativePositionModel::get_cam_translation(float &height, float &rel_x, float &rel_y) const
{
}

} // end namespace firevision
