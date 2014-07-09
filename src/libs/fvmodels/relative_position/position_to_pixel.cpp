/***************************************************************************
 *  globfromrel.cpp - Implementation of the global ball position model
 *
 *  Created: Do Apr 03 16:45:22 2014
 *  Copyright  2014  Tobias Neumann
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

#include "position_to_pixel.h"

#include <utils/math/coord.h>
#include <core/exceptions/software.h>

namespace firevision {
/**
 * @class PositionToPixel
 * Compute a pixel position in the camera image from a cartesian world coordinate.
 */

/**
 * Construct a PositionToPixel model with the required camera geometry
 * @param tf The transform listener used by the calling code
 * @param cam_frame Reference frame of the camera coordinate system
 * @param cam_aperture_x Horizontal opening angle (rad)
 * @param cam_aperture_y Vertical opening angle (rad)
 * @param cam_width_x Horizontal pixel resolution
 * @param cam_height_y Vertical pixel resolution
 * @param cam_angle_y Vertical camera mounting angle
 */
PositionToPixel::PositionToPixel(  fawkes::tf::Transformer* tf, std::string cam_frame,
                              float cam_aperture_x, float cam_aperture_y,
                              unsigned int cam_width_x, unsigned int cam_height_y, float cam_angle_y )
{
  tf_listener = tf;
  cam_frame_  = cam_frame;

  cam_aperture_horizontal_    = cam_aperture_x;
  cam_aperture_vertical_      = cam_aperture_y;
  cam_resulution_horizontal_  = cam_width_x;
  cam_resulution_vertical_    = cam_height_y;

  cam_pixel_per_angle_horizontal_ = double(cam_resulution_horizontal_) / cam_aperture_horizontal_;
  cam_pixel_per_angle_vertical_   = double(cam_resulution_vertical_)   / cam_aperture_vertical_;

  cam_angle_max_horizontal_ = cam_aperture_horizontal_ / 2.0;
  cam_angle_min_horizontal_ = -1.0 * cam_angle_max_horizontal_;

  cam_angle_y_ = cam_angle_y;
  cam_angle_max_vertical_ = cam_angle_y + cam_aperture_vertical_ / 2.0;
  cam_angle_min_vertical_ = cam_angle_y - cam_aperture_vertical_ / 2.0;
}


/**
 * @param   position  the 3dimential position (x, y, z) in the given frame
 * @param   frame     the frame where the data are in
 * @param   time      the timestamp of position
 * @return  the pixel in the camera
 *
 * @throws  OutOfBoundsException
 *          ConnectivityException
 *          ExtrapolationException
 *          LookupException
 */
fawkes::upoint_t
PositionToPixel::get_pixel_position(fawkes::cart_coord_3d_t& position, std::string& frame, const fawkes::Time& time)
{
  fawkes::upoint_t pixel_in_cam;

  fawkes::tf::Stamped<fawkes::tf::Point> coord_in;
  coord_in.frame_id = frame;
  coord_in.stamp    = time;
  coord_in.setX( position.x );
  coord_in.setY( position.y );
  coord_in.setZ( position.z );
  fawkes::tf::Stamped<fawkes::tf::Point> coord_in_transformed;

  //transform frame to cam_frame_ at time
  tf_listener->transform_point(cam_frame_, coord_in, coord_in_transformed);

  //calculate into polar
  fawkes::polar_coord_3d_t polar_in;
  fawkes::cart2polar3d( coord_in_transformed.getX(), coord_in_transformed.getY(), coord_in_transformed.getZ(),
                        (polar_in.phi), (polar_in.theta), (polar_in.r) );

  //calculate into pixel
  if (polar_in.phi   <= cam_angle_min_horizontal_ || polar_in.phi   >= cam_angle_max_horizontal_) {
    throw fawkes::OutOfBoundsException("horizontal position outside cam viewport.", polar_in.phi,
      cam_angle_min_horizontal_, cam_angle_max_horizontal_);
  } else if (polar_in.theta <= cam_angle_min_vertical_   || polar_in.theta >= cam_angle_max_vertical_) {
    throw fawkes::OutOfBoundsException("vertical position outside cam viewport.", polar_in.theta,
      cam_angle_min_vertical_, cam_angle_max_vertical_);
  } else {
    fawkes::point_t pixel_in_cam_rel;
    pixel_in_cam_rel.x = -1.0 * polar_in.phi   * cam_pixel_per_angle_horizontal_;
    pixel_in_cam_rel.y = (polar_in.theta - cam_angle_y_) * cam_pixel_per_angle_vertical_;

    pixel_in_cam.x = pixel_in_cam_rel.x + ( cam_resulution_horizontal_ / 2 );
    pixel_in_cam.y = pixel_in_cam_rel.y + ( cam_resulution_vertical_ / 2 );

    return pixel_in_cam;
  }
}
}
