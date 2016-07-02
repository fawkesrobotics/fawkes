
/***************************************************************************
 *  720to360.h - Laser data data filter to downsample 720 to 360 values
 *
 *  Created: Tue Jun 23 14:36:12 2009
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

#ifndef __PLUGINS_LASER_FILTER_FILTERS_720TO360_H_
#define __PLUGINS_LASER_FILTER_FILTERS_720TO360_H_

#include "filter.h"

class Laser720to360DataFilter : public LaserDataFilter
{
 public:
	Laser720to360DataFilter(const std::string filter_name,
	                        bool average, unsigned int in_data_size,
                          std::vector<LaserDataFilter::Buffer *> &in);
  void filter();

 private:
  bool __average;
};

#endif
