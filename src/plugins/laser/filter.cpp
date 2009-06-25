
/***************************************************************************
 *  filter.cpp - Laser data filter interface
 *
 *  Created: Fri Oct 10 17:12:29 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include "filter.h"
#include <cstdlib>

/** @class LaserDataFilter "filter.h"
 * Laser data filter.
 * With this interface laser filter are described. These filters take laser
 * readings as input, mangle them and return a new array of filtered laser data.
 * @author Tim Niemueller
 *
 * @fn void LaserDataFilter::filter(const float *data, unsigned int data_size) = 0
 * Filter the incoming data.
 * Function shall create the _filtered_data float array with the same size as
 * the incoming data and write filtered data to this interface or copy through
 * the original value if the filter does not apply.
 * @param data the laser data
 * @param data_size the number of elements in the data array
 */

/** @var LaserDataFilter::_filtered_data
 * Allocate a float array and assign your filtered values or copy through the
 * original values if unmodified.
 */

/** @var LaserDataFilter::_filtered_data_size
 * Size in bytes of _filtered_data.
 */

/** @var LaserDataFilter::_free_filtered_data
 * True to have _filtered_data deleted automatically on destruction.
 */

/** Constructor. */
LaserDataFilter::LaserDataFilter()
{
  _filtered_data      = NULL;
  _filtered_data_size = 0;
  _free_filtered_data = true;
}


/** Virtual empty destructor. */
LaserDataFilter::~LaserDataFilter()
{
  if (_filtered_data && _free_filtered_data)  free(_filtered_data);
}


/** Get filtered data array
 * @return a float array of the same size as the last array given to filter()
 * or NULL if filter() was never called.
 */
float *
LaserDataFilter::filtered_data()
{
  return _filtered_data;
}


/** Get size of filtered data array
 * @return size of filtered data array or 0 if filter() was never called.
 */
unsigned int
LaserDataFilter::filtered_data_size()
{
  return _filtered_data_size;
}


/** Get filtered data array and size.
 * @param data upon return contains pointer to filtered data
 * @param data_size upon return contains data size
 */
void
LaserDataFilter::filtered_data(float *&data, unsigned int &data_size)
{
  data      = _filtered_data;
  data_size = _filtered_data_size;
}
