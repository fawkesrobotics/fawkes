
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
#include <cstdlib>
#include <algorithm>

using namespace fawkes;

/** @class LaserCircleSectorDataFilter "circle.h"
 * Erase beams outside specified circle sector.
 * Only data inside the specified circle sector is copied, all other data is
 * set to zero.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param from start angle (index in data)
 * @param to end angle (index in data)
 */
LaserCircleSectorDataFilter::LaserCircleSectorDataFilter(unsigned int from,
							 unsigned int to)
{
  __from = from;
  __to   = to;
}

void
LaserCircleSectorDataFilter::filter(const float *data, unsigned int data_size)
{
  if ( _filtered_data_size != data_size ) {
    if (_filtered_data)  free(_filtered_data);
    _filtered_data      = (float *)malloc(sizeof(float) * data_size);
    _filtered_data_size = data_size;
    for (unsigned int i = 0; i < data_size; ++i) {
      _filtered_data[i] = 0;
    }
  }

  if (__from > __to) {
    for (unsigned int i = __from; i < data_size; ++i) {
      _filtered_data[i] = data[i];
    }
    for (unsigned int i = 0; i <= std::min(__to, data_size-1); ++i) {
      _filtered_data[i] = data[i];
    }
  } else {
    for (unsigned int i = __from; i <= std::min(__to, data_size-1); ++i) {
      _filtered_data[i] = data[i];
    }
  }
}
