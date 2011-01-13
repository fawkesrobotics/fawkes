
/***************************************************************************
 *  mono_drawer.h - Drawer allows to draw arbitrarily in a buffer
 *
 *  Generated: Wed Feb 08 20:30:00 2006
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

#ifndef __FIREVISION_FVUTILS_MONO_DRAWER_H_
#define __FIREVISION_FVUTILS_MONO_DRAWER_H_

#include <fvutils/color/yuv.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class MonoDrawer {

 public:
  MonoDrawer();
  ~MonoDrawer();

  void draw_circle(int center_x, int center_y, unsigned int radius);

  void draw_rectangle(unsigned int x, unsigned int y,
		     unsigned int w, unsigned int h);

  void draw_rectangle_inverted(unsigned int x, unsigned int y,
			     unsigned int w, unsigned int h);

  void draw_point(unsigned int x, unsigned int y);
  void draw_line(unsigned int x_start, unsigned int y_start,
		unsigned int x_end, unsigned int y_end);
  void draw_cross(unsigned int x_center, unsigned int y_center, unsigned int width);

  void set_buffer(unsigned char *buffer,
		 unsigned int width, unsigned int height);

  void set_brightness(unsigned char b);
  void set_overlap(bool o);

 private:
  unsigned char  *__buffer;
  unsigned int    __width;
  unsigned int    __height;
  unsigned char   __brightness;
  bool		  __overlap;

};

} // end namespace firevision

#endif
