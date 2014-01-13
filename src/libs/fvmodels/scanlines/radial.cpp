
/***************************************************************************
 *  radial.cpp - Implementation of the radial scanline model
 *
 *  Created: Tue Jul 19 12:46:52 2005
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

#include <fvmodels/scanlines/radial.h>

#include <utils/system/console_colors.h>

#include <cmath>
#include <cstring>

using fawkes::upoint_t;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ScanlineRadial <fvmodels/scanlines/radial.h>
 * Radial scanlines.
 * Uses circles to generate scanline points. A dead radius is ignored in the
 * center of the image (for example for the camera itself in an omni-vision system).
 * From there circles are used in radius_increment distances. On each circle points are
 * generated in a distance of about step pixels. This is done up to a given maximum
 * radius. If no maximum radius is supplied (max_radius=0) it is automatically
 * calculated depending on the image size.
 */

/** Constructor.
 * @param width image width
 * @param height image height
 * @param center_x radial center center x
 * @param center_y radial center center y
 * @param radius_increment radius increment
 * @param step step
 * @param max_radius maximum radius, if set to 0 will be calculated
 * automatically depending on the image dimensions.
 * @param dead_radius inner radius to ignore
 */
ScanlineRadial::ScanlineRadial(unsigned int width, unsigned int height,
                               unsigned int center_x, unsigned int center_y,
                               unsigned int radius_increment,
                               unsigned int step,
                               unsigned int max_radius, unsigned int dead_radius
                               )
{
  this->width            = width;
  this->height           = height;
  this->center_x         = center_x;
  this->center_y         = center_y;
  this->radius_increment = radius_increment;
  this->step             = step;
  this->dead_radius      = dead_radius;
  this->max_radius       = max_radius;
  this->auto_max_radius  = (max_radius == 0);

  reset();
}

upoint_t
ScanlineRadial::operator*()
{
  return coord;
}

upoint_t*
ScanlineRadial::operator->()
{
  return &coord;
}

upoint_t *
ScanlineRadial::operator++()
{

  if ( done ) return &coord;

  bool ok = false;

  do {

    tmp_x = 0;
    tmp_y = 0;

    if ( current_radius == 0 ) {
      // Special case, after first reset
      current_radius += radius_increment;
      x = 0;
      y = current_radius;
      ok = true;
    } else {

      if ( x < y ) {

        switch (sector) {
        case 0:
          tmp_x = x;
          tmp_y = -y;
          break;

        case 1:
          tmp_x = y;
          tmp_y = -x;
          break;

        case 2:
          tmp_x = y;
          tmp_y = x;
          break;

        case 3:
          tmp_x = x;
          tmp_y = y;
          break;

        case 4:
          tmp_x = -x;
          tmp_y = y;
          break;

        case 5:
          tmp_x = -y;
          tmp_y = x;
          break;

        case 6:
          tmp_x = -y;
          tmp_y = -x;
          break;

        case 7:
          tmp_x = -x;
          tmp_y = -y;
          break;

        default:
          tmp_x = 0;
          tmp_y = 0;
          break;

        }

        x += step;
        y = (int)(sqrt( (float(current_radius * current_radius) - float(x * x)) ) + 0.5);

        ok = true;

      } else {
        //      cout << "x !< y" << endl;
        if (sector == 7) {
          // Need to go to next circle
          current_radius += radius_increment;
          x = 0;
          y = current_radius;
          sector = 0;
          if (current_radius >= max_radius) { done = true; ok = true; }
        } else {
          sector += 1;
          x = 0;
          y = current_radius;
        }
      }

    }

    if ( (tmp_x < -(int)center_x) ||
         (tmp_x > (int)(width - center_x)) ||
         (tmp_y < -(int)center_y) ||
         (tmp_y > (int)(height - center_y))
         ) {
      coord.x = 0;
      coord.y = 0;
      // out of image, not ok
      ok = false;
      //done = true;
    } else {
      coord.x = center_x + tmp_x;
      coord.y = center_y + tmp_y;
    }

  } while (! ok);

  return &coord;
}

upoint_t *
ScanlineRadial::operator++(int)
{
  memcpy(&tmp_coord, &coord, sizeof(upoint_t));
  return &tmp_coord;
}

bool
ScanlineRadial::finished()
{
  return done;
}


/** Do a simple sort of the given array, sorted descending, biggest first
 * this sort is stable
 */
void
ScanlineRadial::simpleBubbleSort(unsigned int array[], unsigned int num_elements)
{
  bool modified = false;
  unsigned int end = num_elements;
  unsigned int tmp;
  do {
    modified = false;

    for (unsigned int i = 0; i < end-1; ++i) {
      if ( array[i] < array[i+1] ) {
        tmp        = array[i];
        array[i]   = array[i+1];
        array[i+1] = tmp;
        end -= 1;
        modified = true;
      }
    }

  } while ( modified );
}

void
ScanlineRadial::reset()
{
  current_radius = radius_increment;
  while (current_radius < dead_radius) {
    current_radius += radius_increment;
  }
  x = 0;
  y = current_radius;
  sector = 0;

  coord.x = center_x;
  coord.y = center_y;

  if ( auto_max_radius ) {
    // Calculate distances to corners of image
    unsigned int dists[4];
    dists[0] = (unsigned int)sqrt( float(center_x * center_x) + float(center_y * center_y) );
    dists[1] = (unsigned int)sqrt( float((width - center_x) * (width - center_x)) + float(center_y * center_y) );
    dists[2] = (unsigned int)sqrt( float((width - center_x) * (width - center_x)) + float((height - center_y) * (height - center_y)) );
    dists[3] = (unsigned int)sqrt( float(center_x * center_x) + float((height - center_y) * (height - center_y)) );

    // now the maximum corner distance is the maximum radius
    simpleBubbleSort(dists, 4);
    max_radius = dists[0] - 1;
  }

  done = false;

  if (radius_increment > max_radius) {
    // cout << msg_prefix << cred << "radius_increment > max_radius, resetting radius_increment to one!" << cnormal << endl;
    radius_increment = 1;
  }

  if (dead_radius > max_radius) {
    // cout << msg_prefix << cred << "dead_radius > max_radius, resetting dead_radius to zero!" << cnormal << endl;
    dead_radius = 0;
    current_radius = radius_increment;
  }

}

const char *
ScanlineRadial::get_name()
{
  return "ScanlineModel::Radial";
}


unsigned int
ScanlineRadial::get_margin()
{
  return radius_increment;
}


/** Set new center point.
 * Sets new center point to move around the scanlines in the image.
 * Does an implicit reset().
 * @param center_x x coordinate of the new center
 * @param center_y y coordinate of the new center
 */
void
ScanlineRadial::set_center(unsigned int center_x, unsigned int center_y)
{
  this->center_x = center_x;
  this->center_y = center_y;
  reset();
}


/** Set new radius.
 * Sets the new maximum and dead radius. Does an implicit reset().
 * @param dead_radius new dead radius
 * @param max_radius new maximum radius, if set to 0 this is automatically
 * calculated depending on the image size.
 */
void
ScanlineRadial::set_radius(unsigned int dead_radius, unsigned int max_radius)
{
  this->max_radius      = max_radius;
  this->dead_radius     = dead_radius;
  this->auto_max_radius = (max_radius == 0);

  reset();
}

} // end namespace firevision
