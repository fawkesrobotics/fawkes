
/***************************************************************************
 *  720to360.cpp - Laser data data filter to downsample 720 to 360 values
 *
 *  Created: Tue Jun 23 14:37:36 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include "720to360.h"

#include <core/exception.h>
#include <utils/math/angle.h>
#include <utils/time/time.h>
#include <cstdlib>

/** @class Laser720to360DataFilter "720to360.h"
 * Downsample filter from 720 to 360 values.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param filter_name name of this filter instance
 * @param average if true, beams will be averaged by left and right neighbours,
 * otherwise every second beam will be used
 * @param in_data_size number of entries input value arrays
 * @param in vector of input arrays
 */
Laser720to360DataFilter::Laser720to360DataFilter(const std::string filter_name,
                                                 bool average,
                                                 unsigned int in_data_size,
                                                 std::vector<LaserDataFilter::Buffer *> &in)
	: LaserDataFilter(filter_name, in_data_size, in, in.size())
{
  if (in_data_size != 720) {
    throw fawkes::Exception("720to360 filter needs input array size of "
			    "720 entries");
  }
  set_out_data_size(360);
  __average = average;
}

void
Laser720to360DataFilter::filter()
{
  const unsigned int vecsize = std::min(in.size(), out.size());
  for (unsigned int a = 0; a < vecsize; ++a) {
    out[a]->frame = in[a]->frame;
    out[a]->timestamp->set_time(in[a]->timestamp);
    float *inbuf  = in[a]->values;
    float *outbuf = out[a]->values;

    if (__average) {
      outbuf[0] = (inbuf[719] + inbuf[0]) / 2.0;
      for (unsigned int i = 1; i < 360; ++i) {
	outbuf[i] = (inbuf[i * 2 - 1] + inbuf[i * 2 + 1]) / 2.0;
      }
    } else {
      for (unsigned int i = 0; i < 360; ++i) {
	outbuf[i] = inbuf[i * 2];
      }
    }
  }
}
