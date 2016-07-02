
/***************************************************************************
 *  reverse_angle.cpp - Reverse the angle in which laser data is taken
 *
 *  Created: Wed Jan 06 17:15:38 2010
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

#include "reverse_angle.h"

#include <core/exception.h>
#include <utils/math/angle.h>
#include <utils/time/time.h>
#include <cstdlib>

/** @class LaserReverseAngleDataFilter "reverse_angle.h"
 * Reverse the angle of beams.
 * This filter will reverse the direction in which the beams are stored.
 * If the original interface stores the data in clockwise direction, the
 * outcome will be in counter-clockwise direction and vice versa. This is
 * required for example to convert between the (clockwise) RCSoftX angles,
 * and the (counter-clockwise) angles in the Fawkes coordinate system.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param filter_name name of this filter instance
 * @param in_data_size number of entries input value arrays
 * @param in vector of input arrays
 */
LaserReverseAngleDataFilter::LaserReverseAngleDataFilter(const std::string filter_name,
                                                         unsigned int in_data_size,
                                                         std::vector<LaserDataFilter::Buffer *> &in)
	: LaserDataFilter(filter_name, in_data_size, in, in.size())
{
}

void
LaserReverseAngleDataFilter::filter()
{
  const unsigned int vecsize = std::min(in.size(), out.size());
  const unsigned int arrsize = std::min(in_data_size, out_data_size);
  for (unsigned int a = 0; a < vecsize; ++a) {
    out[a]->frame = in[a]->frame;
    out[a]->timestamp->set_time(in[a]->timestamp);
    float *inbuf  = in[a]->values;
    float *outbuf = out[a]->values;
    for (unsigned int i = 0; i < arrsize; ++i) {
      outbuf[i] = inbuf[arrsize - i];
    }
  }
}
