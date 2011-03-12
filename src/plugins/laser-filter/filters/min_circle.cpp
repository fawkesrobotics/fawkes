
/***************************************************************************
 *  min_circle.cpp - Laser data min circle data filter
 *
 *  Created: Sat Feb 19 00:23:27 2011
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

#include "min_circle.h"

#include <utils/math/angle.h>
#include <cstdlib>

/** @class LaserMinCircleDataFilter "circle.h"
 * Erase beams below a certain minimum distance distance.
 * All beams shorter than a given radius are erase (set to 0).
 * @author Tim Niemueller
 */

/** Constructor.
 * @param radius radius of cut-off circle in meters
 */
LaserMinCircleDataFilter::LaserMinCircleDataFilter(float radius)
{
  __radius = radius;
}

void
LaserMinCircleDataFilter::filter(const float *data, unsigned int data_size)
{
  if ( _filtered_data_size != data_size ) {
    if (_filtered_data)  free(_filtered_data);
    _filtered_data      = (float *)malloc(sizeof(float) * data_size);
    _filtered_data_size = data_size;
  }

  for (unsigned int i = 0; i < data_size; ++i) {
    if (data[i] < __radius) {
      _filtered_data[i] = 0;
    } else {
      _filtered_data[i] = data[i];
    }
  }
}
