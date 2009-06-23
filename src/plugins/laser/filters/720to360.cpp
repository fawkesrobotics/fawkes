
/***************************************************************************
 *  720to360.cpp - Laser data data filter to downsample 720 to 360 values
 *
 *  Created: Tue Jun 23 14:37:36 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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
#include <cstdlib>

/** @class Laser720to360DataFilter "720to360.h"
 * Downsample filter from 720 to 360 values.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param average if true, beams will be averaged by left and right neighbours,
 * otherwise every second beam will be used
 */
Laser720to360DataFilter::Laser720to360DataFilter(bool average)
{
  __average      = average;
  _filtered_data = (float *)malloc(sizeof(float) * 360);
}

void
Laser720to360DataFilter::filter(const float *data, unsigned int data_size)
{
  if ( data_size != 720 ) {
    throw fawkes::Exception("Expected 720 values, but got %u", data_size);
  }

  if (__average) {
    _filtered_data[0] = (data[719] / data[0]) / 2.0;
    for (unsigned int i = 1; i < 360; ++i) {
      _filtered_data[i] = (data[i * 2 - 1] + data[i * 2 + 1]) / 2.0;
    }
  } else {
    for (unsigned int i = 0; i < 360; ++i) {
      _filtered_data[i] = data[i * 2];
    }
  }
}
