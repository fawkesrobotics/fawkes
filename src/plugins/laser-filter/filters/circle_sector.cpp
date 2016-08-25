
/***************************************************************************
 *  circle_sector.cpp - Filter laser data for circle sector
 *
 *  Created: Sat Feb 19 00:28:41 2011
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

#include "circle_sector.h"

#include <core/exception.h>
#include <utils/math/angle.h>
#include <utils/time/time.h>
#include <algorithm>
#include <cstring>

using namespace fawkes;

/** @class LaserCircleSectorDataFilter "circle.h"
 * Erase beams outside specified circle sector.
 * Only data inside the specified circle sector is copied, all other data is
 * set to zero.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param filter_name name of this filter instance
 * @param from start angle (index in data)
 * @param to end angle (index in data)
 * @param in_data_size number of entries in value arrays
 * @param in vector of input arrays
 */
LaserCircleSectorDataFilter::LaserCircleSectorDataFilter(const std::string filter_name,
                                                         unsigned int from,
                                                         unsigned int to,
                                                         unsigned int in_data_size,
                                                         std::vector<LaserDataFilter::Buffer *> &in)
	: LaserDataFilter(filter_name, in_data_size, in, in.size())
{
  __from = from;
  __to   = to;
}


void
LaserCircleSectorDataFilter::filter()
{
  const unsigned int vecsize = std::min(in.size(), out.size());
  const unsigned int arrsize = std::min(in_data_size, out_data_size);
  for (unsigned int a = 0; a < vecsize; ++a) {

    reset_outbuf(out[a]);
    out[a]->frame = in[a]->frame;
    out[a]->timestamp->set_time(in[a]->timestamp);

    float *inbuf  = in[a]->values;
    float *outbuf = out[a]->values;

    if (__from > __to) {
      for (unsigned int i = __from; i < arrsize; ++i) {
	outbuf[i] = inbuf[i];
      }
      for (unsigned int i = 0; i <= std::min(__to, arrsize-1); ++i) {
	outbuf[i] = inbuf[i];
      }
    } else {
      for (unsigned int i = __from; i <= std::min(__to, arrsize-1); ++i) {
	outbuf[i] = inbuf[i];
      }
    }
  }
}
