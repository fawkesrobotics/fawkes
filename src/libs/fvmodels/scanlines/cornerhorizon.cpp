
/***************************************************************************
 *  cornerhorizon.cpp - Implementation of the corner horizon
 *
 *  Created: Fri Apr 07 04:37:25 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *             2006       Stefan Schiffer
 *             2006       Christoph Mies
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

#include <fvmodels/scanlines/cornerhorizon.h>
#include <utils/math/angle.h>
#include <cstdlib>
#include <cstring>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

const float CornerHorizon::M_PI_HALF = M_PI / 2.f;

/** @class CornerHorizon <fvmodels/scanlines/cornerhorizon.h>
 * Cut of arbitrary scanline models at an artificial horizon.
 * The artificial horizon is calculated by the highest corner that is visible
 * in the image. From that the Y coordinate in the image is used and everything
 * above that point is ignored from the scanline grid.
 *
 * This class was written in a one-night hacking sensation at RoboLudens 2006
 * in Eindhoven. For that time it is pretty readable code and we are using it
 * since then. Cool!
 *
 * @author Tim Niemueller
 * @author Stefan Schiffer
 * @author Christoph Mies
 */

/** Constructor.
 * @param model Model to apply the artificial horizon on. This model is deleted on
 * the destruction of the CornerHorizon instance so you can forget about it in the
 * using application.
 * @param field_length length of soccer field
 * @param field_width width of soccer field
 * @param field_border size of border around the field (i.e. distance between the
 * outer white line and the physical field end)
 * @param image_width image width in pixels
 * @param image_height image height in pixels
 * @param camera_height height of camera above ground
 * @param camera_ori orientation of camera on the robot in degrees
 * @param horizontal_angle horizontal viewing angle in degrees
 * @param vertical_angle vertical viewing angle in degrees
 */
CornerHorizon::CornerHorizon(ScanlineModel *model,
           float field_length, float field_width, float field_border,
           unsigned int image_width, unsigned int image_height,
           float camera_height, float camera_ori,
           float horizontal_angle, float vertical_angle
           )
{
  this->model = model;

  this->field_length = field_length;
  this->field_width  = field_width;
  this->field_border = field_border;

  this->image_width        = image_width;
  this->image_height       = image_height;
  this->horizontal_angle   = deg2rad( horizontal_angle );
  this->vertical_angle     = deg2rad( vertical_angle   );
  this->camera_ori         = deg2rad( camera_ori       );
  this->camera_height      = camera_height;

  pan_pixel_per_rad  = this->image_width  / this->horizontal_angle;
  tilt_pixel_per_rad = this->image_height / this->vertical_angle;

  calculated = false;

  coord.x = coord.y = 0;
}


/** Destructor.
 * Not that this deletes the supplied model!
 */
CornerHorizon::~CornerHorizon()
{
  delete model;
}


upoint_t
CornerHorizon::operator*()
{
  return coord;
}


upoint_t*
CornerHorizon::operator->()
{
  return &coord;
}


/** Calculate horizon point. */
void
CornerHorizon::calculate()
{

  float phi = normalize_mirror_rad( pose_ori + pan );

  float corner_x, corner_y;

  if ( (phi > 0) && (phi <= M_PI_HALF) ) {
    corner_x = field_length / 2 + field_border;
    corner_y = field_width  / 2 + field_border;
  } else if ( (phi > M_PI_HALF) && (phi <= M_PI) ) {
    corner_x = - (field_length / 2 + field_border );
    corner_y = field_width / 2 + field_border;
  } else if ( (phi <= 0) && (phi > - M_PI_HALF) ) {
    corner_x = field_length / 2 + field_border;
    corner_y = - (field_width / 2 + field_border);
  } else /* if (phi <= - M_PI_HALF) */ {
    corner_x = - (field_length / 2 + field_border );
    corner_y = - (field_width / 2 + field_border);
  }

  float d_x = corner_x - pose_x;
  float d_y = corner_y - pose_y;

  float d = sqrt( d_x * d_x + d_y * d_y );

  float alpha = atan2f( d, camera_height );
  float beta  = M_PI_HALF - alpha;

  int hor = (int)round((beta + tilt) * tilt_pixel_per_rad);

  if ((unsigned int)abs(hor) >= (image_height / 2)) {
    if ( hor < 0 ) {
      hor = - ( image_height / 2 );
    } else {
      hor = image_height / 2;
    }
  }

  horizon = image_height / 2 + hor;

  /*
  cout << "Calculated: " << endl
       << "  phi=" << phi << endl
       << "  corner_x=" << corner_x << endl
       << "  corner_y=" << corner_y << endl
       << "  d_x=" << d_x << endl
       << "  d_y=" << d_y << endl
       << "  d=" << d << endl
       << "  alpha=" << alpha << endl
       << "  beta=" << beta << endl
       << "  hor=" << hor << endl
       << "  horizon=" << horizon << endl
       << "  pan_pixel_per_rad=" << pan_pixel_per_rad << endl
       << "  tilt_pixel_per_rad=" << tilt_pixel_per_rad << endl;
  */

}


upoint_t *
CornerHorizon::operator++()
{
  if ( ! calculated) {
    calculate();
    calculated = true;
  }

  coord.x = (*model)->x;
  coord.y = (*model)->y;

  do {
    ++(*model);
  } while ( ((*model)->y < horizon) && ( ! model->finished()) );

  if ( ((*model)->y < horizon) || model->finished() ) {
    // finished
    //cout << "1 (" << coord.x << "," << coord.y << ")" << endl;
    return &coord;
  } else {
    coord.x = (*model)->x;
    coord.y = (*model)->y;
    //cout << "2 (" << coord.x << "," << coord.y << ")" << endl;
    return &coord;
  }
}


upoint_t *
CornerHorizon::operator++(int)
{
  if ( ! calculated) {
    calculate();
    calculated = true;
  }
  memcpy(&tmp_coord, &coord, sizeof(upoint_t));

  do {
    ++(*model);
  } while ( ((*model)->y < horizon) && ! model->finished() );

  if ( ((*model)->y >= horizon) && ! model->finished() ) {
    coord.x = (*model)->x;
    coord.y = (*model)->y;
    //cout << "3 (" << coord.x << "," << coord.y << ")" << endl;
  }

  return &tmp_coord;
}


bool
CornerHorizon::finished()
{
  return model->finished();
}


void
CornerHorizon::reset()
{
  calculated = false;
  coord.x = coord.y = 0;
  model->reset();
}


const char *
CornerHorizon::get_name()
{
  return "ScanlineModel::CornerHorizon";
}


unsigned int
CornerHorizon::get_margin()
{
  return model->get_margin();
}


/** Get the horizon point.
 * @return y coordinate of the horizon point.
 */
unsigned int
CornerHorizon::getHorizon()
{
  return horizon;
}


void
CornerHorizon::set_robot_pose(float x, float y, float ori)
{
  pose_x = x;
  pose_y = y;
  pose_ori = ori;
}


void
CornerHorizon::set_pan_tilt(float pan, float tilt)
{
  this->pan  = pan;
  this->tilt = tilt;
}

} // end namespace firevision
