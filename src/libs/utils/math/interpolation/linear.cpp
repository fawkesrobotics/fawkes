
/***************************************************************************
 *  linear.cpp - Linear interpolator
 *
 *  Created: Tue Nov 18 11:13:13 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *             2008  Graeme McPhillips
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

#include <utils/math/interpolation/linear.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LinearInterpolator <utils/math/interpolation/linear.h>
 * Linear value interpolator.
 * The interpolator creates intermediate points given a starting and and
 * end point and time constraints. Times are supplied in a discrete unit like
 * miliseconds or microseconds.
 * The values are interpolated on a straight line between the starting and the
 * end point.
 *
 * The calculation is executed with the following equation:
 * \f[
 *   \frac{t_\mathrm{current}}{t_\mathrm{end}} \cdot (v_\mathrm{end} - v_\mathrm{start}) + v_\mathrm{start}
 * \f]
 *
 * @author Tim Niemueller
 * @author Graeme McPhillips
 * @author Stephen Marais
 */

float
LinearInterpolator::interpolate(float t_current, float t_end, float t_step,
				float v_start, float v_end)
{
  return (t_current / t_end) * (v_end - v_start) + v_start;
}


} // end namespace fawkes

