
/***************************************************************************
 *  mono_drawer.cpp - Utility to draw in a buffer
 *
 *  Generated: Wed Feb 08 20:55:38 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *             2010  Bahram Maleki-Fard
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

#include <fvutils/draw/mono_drawer.h>
#include <fvutils/color/yuv.h>

#include <cmath>
#include <algorithm>
#include <unistd.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MonoDrawer <fvutils/draw/mono_drawer.h>
 * Draw to a monochrome image.
 * @author Tim Niemueller (Base)
 * @author Bahram Maleki-Fard (Modification)
 */

/** Constructor. */
MonoDrawer::MonoDrawer()
{
  __buffer = NULL;
  __brightness = 1;
  __overlap = 1;
}

/** Destructor */
MonoDrawer::~MonoDrawer()
{
}


/** Set the buffer to draw to
 * @param buffer buffer to draw to, must be MONO8 formatted. E.g. Y-plane of YUV
 * @param width width of the buffer
 * @param height height of the buffer
 */
void
MonoDrawer::set_buffer(unsigned char *buffer,
		  unsigned int width, unsigned int height)
{
  this->__buffer     = buffer;
  this->__width      = width;
  this->__height     = height;
}


/** Set drawing brightness.
 * @param b brightness; 0-255
 */
void
MonoDrawer::set_brightness(unsigned char b)
{
  __brightness = b;
}


/** Enable/Disable transparency (overlapping pixels increase brightness).
 * @param o overlapping true/false
 */
void
MonoDrawer::set_overlap(bool o)
{
  __overlap = o;
}


/** Draw circle.
 * Draws a circle at the given center point and with the given radius.
 * @param center_x x coordinate of circle center
 * @param center_y y coordinate of circle center
 * @param radius radius of circle
 */
void
MonoDrawer::draw_circle(int center_x, int center_y, unsigned int radius)
{
  if (__buffer == NULL) return;

  unsigned int x  = 0,
               y  = radius,
               r2 = radius * radius;

  unsigned int x_tmp, y_tmp, ind_tmp;

  while (x <= y) {

    x_tmp = center_x + x;
    y_tmp = center_y + y;
    if ( (x_tmp < __width) && (y_tmp < __height) ) {
      ind_tmp = y_tmp * __width + x_tmp;
      if( __overlap )
        __buffer[ind_tmp]   = std::min(255, __buffer[ind_tmp] + __brightness);
      else
        __buffer[ind_tmp]   = __brightness;
      ind_tmp /= 2;
    }

    x_tmp = center_x - x;
    y_tmp = center_y + y;
    if ( (x_tmp < __width) && (y_tmp < __height) ) {
      ind_tmp = y_tmp * __width + x_tmp;
      if( __overlap )
        __buffer[ind_tmp]   = std::min(255, __buffer[ind_tmp] + __brightness);
      else
        __buffer[ind_tmp]   = __brightness;
      ind_tmp /= 2;
    }

    x_tmp = center_x + y;
    y_tmp = center_y + x;
    if ( (x_tmp < __width) && (y_tmp < __height) ) {
      ind_tmp = y_tmp * __width + x_tmp;
      if( __overlap )
        __buffer[ind_tmp]   = std::min(255, __buffer[ind_tmp] + __brightness);
      else
        __buffer[ind_tmp]   = __brightness;
      ind_tmp /= 2;
    }

    x_tmp = center_x - y;
    y_tmp = center_y + x;
    if ( (x_tmp < __width) && (y_tmp < __height) ) {
      ind_tmp = y_tmp * __width + x_tmp;
      if( __overlap )
        __buffer[ind_tmp]   = std::min(255, __buffer[ind_tmp] + __brightness);
      else
        __buffer[ind_tmp]   = __brightness;
      ind_tmp /= 2;
    }

    x_tmp = center_x + x;
    y_tmp = center_y - y;
    if ( (x_tmp < __width) && (y_tmp < __height) ) {
      ind_tmp = y_tmp * __width + x_tmp;
      if( __overlap )
        __buffer[ind_tmp]   = std::min(255, __buffer[ind_tmp] + __brightness);
      else
        __buffer[ind_tmp]   = __brightness;
      ind_tmp /= 2;
    }

    x_tmp = center_x - x;
    y_tmp = center_y - y;
    if ( (x_tmp < __width) && (y_tmp < __height)) {
      ind_tmp = y_tmp * __width + x_tmp;
      if( __overlap )
        __buffer[ind_tmp]   = std::min(255, __buffer[ind_tmp] + __brightness);
      else
        __buffer[ind_tmp]   = __brightness;
      ind_tmp /= 2;
    }

    x_tmp = center_x + y;
    y_tmp = center_y - x;
    if ( (x_tmp < __width) && (y_tmp < __height)) {
      ind_tmp = y_tmp * __width + x_tmp;
      if( __overlap )
        __buffer[ind_tmp]   = std::min(255, __buffer[ind_tmp] + __brightness);
      else
        __buffer[ind_tmp]   = __brightness;
      ind_tmp /= 2;
    }

    x_tmp = center_x - y;
    y_tmp = center_y - x;
    if ( (x_tmp < __width) && (y_tmp < __height) ) {
      ind_tmp = y_tmp * __width + x_tmp;
      if( __overlap )
        __buffer[ind_tmp]   = std::min(255, __buffer[ind_tmp] + __brightness);
      else
        __buffer[ind_tmp]   = __brightness;
      ind_tmp /= 2;
    }

    ++x;
    y=(int)(sqrt((float)(r2 - x * x))+0.5);
  }

}

} // end namespace firevision
