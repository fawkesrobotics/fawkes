
/***************************************************************************
 *  circle_sector.h - Filter laser data for circle sector
 *
 *  Created: Sat Feb 19 00:27:33 2011
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

#ifndef __PLUGINS_LASER_FILTER_FILTERS_CIRCLE_SECTOR_H_
#define __PLUGINS_LASER_FILTER_FILTERS_CIRCLE_SECTOR_H_

#include "filter.h"

class LaserCircleSectorDataFilter : public LaserDataFilter
{
 public:
	LaserCircleSectorDataFilter(const std::string filter_name,
	                            unsigned int from, unsigned int to,
	                            unsigned int data_size, std::vector<LaserDataFilter::Buffer *> &in);

  void filter();

 private:
  unsigned int __from;
  unsigned int __to;
};

#endif
