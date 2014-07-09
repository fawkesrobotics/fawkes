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

#include "thresholds_black.h"
#include <cmath>

namespace firevision
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ColorModelBlack <fvmodels/color/thresholds_black.h>
 * Detect configurable shades/hues of "black" as a cuboid in YUV space.
 */

/**
 * Initialize black colormodel. The Y reference component is always 0,
 * i.e. the accepted cuboid extends from Y=0 to Y=y_thresh, by u_thresh
 * around ref_u, and by v_thresh around ref_v.
 *
 * @param y_thresh maximum brightness
 * @param u_thresh maximum difference from ref_u
 * @param v_thresh maximum difference from ref_v
 * @param ref_u U component of the "black" reference color (default 128)
 * @param ref_v V component of the "black" reference color (default 128)
 */
ColorModelBlack::ColorModelBlack(unsigned int y_thresh, unsigned int u_thresh, unsigned int v_thresh,
  unsigned int ref_u, unsigned int ref_v) :
    y_thresh_(y_thresh),
    u_thresh_(u_thresh),
    v_thresh_(v_thresh),
    ref_u_ (ref_u),
    ref_v_ (ref_v) {}

color_t
ColorModelBlack::determine(unsigned int y,
			   unsigned int u,
			   unsigned int v) const
{
  int diff_u = ref_u_ - u;
  int diff_v = ref_v_ - v;
  if ( y <= y_thresh_
#if defined(__GNUC__) && ((__GNUC__ == 4 && __GNUC_MINOR__ >= 6) || (__GNUC__ > 4))
      && std::abs(diff_u) < u_thresh_ && std::abs(diff_v) < v_thresh_
#else
      && (diff_u < 0) ? (diff_u > -1*(int)u_thresh_) : (diff_u < (int)u_thresh_)
      && (diff_v < 0) ? (diff_v > -1*(int)v_thresh_) : (diff_v < (int)v_thresh_)
#endif
  )
  {
    return C_BLACK;
  } else {
    return C_OTHER;
  }
}

const char *
ColorModelBlack::get_name()
{
  return "ColorModelBlack";
}

} // end namespace firevision
