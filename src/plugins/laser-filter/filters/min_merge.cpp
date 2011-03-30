
/***************************************************************************
 *  min_merge.cpp - Laser min merge data filter
 *
 *  Created: Wed Mar 16 21:46:36 2011
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

#include "min_merge.h"

#include <cstring>

/** @class LaserMinMergeDataFilter "min_merge.h"
 * Merge multiple laser data arrays into one.
 * For each value in the output array takes the minimum value of all input
 * arrays.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param in_data_size number of entries input value arrays
 * @param in vector of input arrays
 */
LaserMinMergeDataFilter::LaserMinMergeDataFilter(unsigned int in_data_size,
						 std::vector<float *> in)
  : LaserDataFilter(in_data_size, in, 1)
{
}


void
LaserMinMergeDataFilter::filter()
{
  const unsigned int vecsize = in.size();
  if (vecsize == 0)  return;

  float *outbuf = out[0];

  memcpy(outbuf, in[0], out_data_size);

  for (unsigned int a = 1; a < vecsize; ++a) {
    float *inbuf  = in[a];
    for (unsigned int i = 0; i < (const unsigned int)out_data_size; ++i) {
      if ( (outbuf[i] == 0) || ((inbuf[i] != 0) && (inbuf[i] < outbuf[i])) ) {
	outbuf[i] = inbuf[i];
      }
    }
  }
}
