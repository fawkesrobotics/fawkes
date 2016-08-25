
/***************************************************************************
 *  cascade.cpp - Laser data filter cascade
 *
 *  Created: Thu Jun 25 01:07:53 2009
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

#include "cascade.h"

/** @class LaserDataFilterCascade "filters/cascade.h"
 * Cascade of several laser filters to one.
 * The filters are executed in the order they are added to the cascade.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param filter_name name of this filter combo
 * @param in_data_size number of entries input value arrays
 * @param in vector of input arrays
 */
LaserDataFilterCascade::LaserDataFilterCascade(const std::string filter_name,
                                               unsigned int in_data_size,
                                               std::vector<Buffer *> &in)
	: LaserDataFilter(filter_name, in_data_size, in, 0)
{
  out_data_size = in_data_size;
  out = in;
  set_array_ownership(false, false);
}


/** Destructor. */
LaserDataFilterCascade::~LaserDataFilterCascade()
{
  delete_filters();
}


/** Set filtered data array
 * @param out vector of output values. The vector is only accepted if it has
 * the same size as the current one. The filter will now longer assume
 * ownership of the arrays in the vector. Either free the memory or call
 * set_array_ownership().
 */
void
LaserDataFilterCascade::set_out_vector(std::vector<LaserDataFilter::Buffer *> &out)
{
  __filters.back()->set_out_vector(out);
  this->out = __filters.back()->get_out_vector();
}


/** Add a filter to the cascade.
 * @param filter filter to add
 */
void
LaserDataFilterCascade::add_filter(LaserDataFilter *filter)
{
  __filters.push_back(filter);
  out_data_size = filter->get_out_data_size();
  out = filter->get_out_vector();
}


/** Remove a filter from the cascade.
 * @param filter filter to remove
 */
void
LaserDataFilterCascade::remove_filter(LaserDataFilter *filter)
{
  __filters.remove(filter);
}


/** Delete all filters. */
void
LaserDataFilterCascade::delete_filters()
{
  for (__fit = __filters.begin(); __fit != __filters.end(); ++__fit) {
    delete *__fit;
  }
  __filters.clear();
}


void
LaserDataFilterCascade::filter()
{
  for (__fit = __filters.begin(); __fit != __filters.end(); ++__fit) {
    (*__fit)->filter();
  }
}
