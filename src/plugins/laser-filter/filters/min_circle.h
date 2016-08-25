
/***************************************************************************
 *  min_circle.h - Laser data min circle radius data filter
 *
 *  Created: Sat Feb 19 00:21:41 2011
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

#ifndef __PLUGINS_LASER_FILTER_FILTERS_MIN_CIRCLE_H_
#define __PLUGINS_LASER_FILTER_FILTERS_MIN_CIRCLE_H_

#include "filter.h"

class LaserMinCircleDataFilter : public LaserDataFilter
{
 public:
	LaserMinCircleDataFilter(const std::string filter_name, float radius,
                           unsigned int data_size, std::vector<LaserDataFilter::Buffer *> &in);

  void filter();

 private:
  float  __radius;
};

#endif
