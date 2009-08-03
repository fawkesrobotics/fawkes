
/***************************************************************************
 *  sinusoidal.cpp - Sinusoidal interpolator
 *
 *  Created: Tue Nov 18 11:27:44 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *             2008  Graeme McPhillips
 *             2008  Stephen Marais
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

#include <utils/math/interpolation/sinusoidal.h>

#include <cmath>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SinusoidalInterpolator <utils/math/interpolation/linear.h>
 * Sinusoidal value interpolator.
 * The interpolator creates intermediate points given a starting and and
 * end point and time constraints. Times are supplied in a discrete unit like
 * miliseconds or microseconds.
 * The values are interpolated on a sinusoidal curve with a slow start, the
 * greatest slope in the middle and then a slow down in the end. This
 * interpolation is useful for example for smooth servo movements.
 *
 * The calculation is executed with the following equation:
 * \f[
 *   \left(\frac{1}{2} \sin\left(\frac{1}{2} + \frac{t_\mathrm{current}}{t_\mathrm{end}} \pi \right) + \frac{1}{2}\right) \cdot (v_\mathrm{end} - v_\mathrm{start}) + v_\mathrm{start}
 * \f]
 *
 * @author Tim Niemueller
 * @author Graeme McPhillips
 * @author Stephen Marais
 */

float
SinusoidalInterpolator::interpolate(float t_current, float t_end, float t_step,
				    float v_start, float v_end)
{
  return ( sin((-0.5 + (t_current / t_end)) * M_PI) / 2.0 + 0.5) * (v_end - v_start) + v_start;
}


} // end namespace fawkes
