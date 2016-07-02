
/***************************************************************************
 *  deadspots.h - Laser data dead spots filter
 *
 *  Created: Wed Jun 24 22:39:19 2009
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

#ifndef __PLUGINS_LASER_FILTER_FILTERS_DEADSPOTS_H_
#define __PLUGINS_LASER_FILTER_FILTERS_DEADSPOTS_H_

#include "filter.h"

#include <vector>
#include <utility>
#include <string>

namespace fawkes {
  class Configuration;
  class Logger;
}

class LaserDeadSpotsDataFilter : public LaserDataFilter
{
 public:
  LaserDeadSpotsDataFilter(const std::string filter_name,
                           fawkes::Configuration *config, fawkes::Logger *logger,
                           std::string prefix,
                           unsigned int data_size, std::vector<LaserDataFilter::Buffer *> &in);
  ~LaserDeadSpotsDataFilter();

  void filter();

 private:
  void calc_spots();
  void set_out_vector(std::vector<LaserDataFilter::Buffer *> &out);


 private:
  fawkes::Logger *__logger;

  unsigned int  __num_spots;
  unsigned int *__dead_spots;
  std::vector<std::pair<float, float> > __cfg_dead_spots;
};

#endif
