
/***************************************************************************
 *  max_circle.cpp - Laser data circle data filter (example)
 *
 *  Created: Fri Oct 10 17:16:57 2008
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "max_circle.h"

#include <utils/math/angle.h>
#include <cstdlib>

/** @class LaserMaxCircleDataFilter "circle.h"
 * Cut of laser data at max distance.
 * All beams longer than a given radius are cut of at the maximum length.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param radius radius of cut-off circle in meters
 * @param in_data_size number of entries input value arrays
 * @param in vector of input arrays
 */
LaserMaxCircleDataFilter::LaserMaxCircleDataFilter(float radius,
						   unsigned int in_data_size,
						   std::vector<float *> in)
  : LaserDataFilter(in_data_size, in, in.size())
{
  __radius = radius;
}

void
LaserMaxCircleDataFilter::filter()
{
  const unsigned int vecsize = std::min(in.size(), out.size());
  const unsigned int arrsize = std::min(in_data_size, out_data_size);
  for (unsigned int a = 0; a < vecsize; ++a) {
    float *inbuf  = in[a];
    float *outbuf = out[a];
    for (unsigned int i = 0; i < arrsize; ++i) {
      if (inbuf[i] > __radius) {
	outbuf[i] = __radius;
      } else {
	outbuf[i] = inbuf[i];
      }
    }
  }
}
