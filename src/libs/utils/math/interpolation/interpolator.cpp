
/***************************************************************************
 *  interpolator.cpp - Interpolator
 *
 *  Created: Tue Nov 18 10:45:18 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include <utils/math/interpolation/interpolator.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Interpolator <utils/math/interpolation/interpolator.h>
 * Value interpolator.
 * The interpolator creates intermediate points given a starting and and
 * end point and time constraints. Times are supplied any chose time scale, it
 * only has to be a linear time measure. Common are miliseconds or seconds.
 * @author Tim Niemueller
 *
 * @fn float Interpolator::interpolate(float t_current, float t_end, float t_step, float v_start, float v_end)
 * Interpolate a point at a specific time.
 * @param t_current current time for which to calculate the intermediate point
 * @param t_end end time/total time. The start time is always 0.
 * @param t_step Time of a time slice for discrete intermediate interpolation
 * points. Set to 1 for maximum resolution.
 * @param v_start start value
 * @param v_end end value
 * @return interpolated value at time t_current between t_start and t_end.
 */

/** Virtual empty descructor. */
Interpolator::~Interpolator()
{
}

} // end namespace fawkes

