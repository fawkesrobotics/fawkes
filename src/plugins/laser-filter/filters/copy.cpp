/***************************************************************************
 *  copy.cpp - Laser data filter to copy data without modification
 *
 *  Created: Mon 16 Apr 2018 13:50:26 CEST 13:50
 *  Copyright  2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "copy.h"

#include <utils/time/time.h>

/** @class LaserCopyDataFilter "copy.h"
 * Copy laser data without modification to a new name.
 * @author Till Hofmann
 */

/** Constructor.
 * @param filter_name name of this filter instance
 * @param in_data_size number of entries in the input value arrays
 * @param in vector of input arrays
 */

LaserCopyDataFilter::LaserCopyDataFilter(const std::string filter_name, uint
    in_data_size, std::vector<Buffer *> &in)
  : LaserDataFilter(filter_name, in_data_size, in, in.size())
{}

void
LaserCopyDataFilter::filter()
{
  const uint num_buffers = std::min(in.size(), out.size());
  const uint data_size = std::min(in_data_size, out_data_size);
  for (uint buffer_i = 0; buffer_i < num_buffers; buffer_i++) {
    out[buffer_i]->frame = in[buffer_i]->frame;
    out[buffer_i]->timestamp->set_time(in[buffer_i]->timestamp);
    float *inbuf = in[buffer_i]->values;
    float *outbuf = out[buffer_i]->values;
    for (uint i = 0; i < data_size; i++) {
      outbuf[i] = inbuf[i];
    }
  }
}
