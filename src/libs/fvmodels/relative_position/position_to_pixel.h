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

#ifndef __FIREVISION_PIXELFROMPOSITION_H_
#define __FIREVISION_PIXELFROMPOSITION_H_

#include <tf/transformer.h>
#include <utils/math/types.h>

#include <string>

namespace firevision {

class PositionToPixel
{
public:
  PositionToPixel(  fawkes::tf::Transformer* tf, std::string cam_frame,
                  float cam_aperture_x, float cam_aperture_y,
                  unsigned int cam_width_x, unsigned int cam_height_y, float cam_angle_y = 0 );

  fawkes::upoint_t get_pixel_position(fawkes::cart_coord_3d_t& position, std::string& frame, const fawkes::Time& time);
  fawkes::point_t get_pixel_position_unchecked(fawkes::cart_coord_3d_t& position, std::string& frame, const fawkes::Time& time);

private:
  std::string cam_frame_;
  fawkes::tf::Transformer* tf_listener;

  float         cam_aperture_horizontal_;
  float         cam_aperture_vertical_;
  float         cam_angle_y_;
  unsigned int  cam_resolution_x_;
  unsigned int  cam_resolution_y_;

  float         cam_pixel_per_angle_horizontal_;
  float         cam_pixel_per_angle_vertical_;

  float         cam_angle_max_horizontal_;
  float         cam_angle_min_horizontal_;
  float         cam_angle_max_vertical_;
  float         cam_angle_min_vertical_;
};
}
#endif /* __FIREVISION_PIXELFROMPOSITION_H_ */
