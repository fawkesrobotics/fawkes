
/***************************************************************************
 *  map_filter.h - Laser map data filter
 *
 *  Created: Fri Jul 17 20:38:14 2015
 *  Copyright  2006-2015  Tim Niemueller [www.niemueller.de]
 *                  2015  Tobias Neumann
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

#ifndef __PLUGINS_LASER_FILTER_FILTERS_MAP_FILTER_H_
#define __PLUGINS_LASER_FILTER_FILTERS_MAP_FILTER_H_

#include <plugins/amcl/amcl_utils.h>
#include <plugins/amcl/map/map.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <aspect/configurable.h>
#include "filter.h"

class LaserMapFilterDataFilter : public LaserDataFilter
{
 private:
  
  fawkes::tf::Transformer *tf_listener_;
  fawkes::Configuration   *config_;
  fawkes::Logger          *logger_;

  map_t                   *map_;
  std::string             frame_map_;
  float                   cfg_occupied_thresh_;

 public:

  LaserMapFilterDataFilter(const std::string filter_name,
                           unsigned int in_data_size,
                           std::vector<LaserDataFilter::Buffer *> &in,
                           fawkes::tf::Transformer * tf_listener,
                           fawkes::Configuration *config,
                           fawkes::Logger *logger);

  virtual void filter();

 private:
  map_t * load_map();
  bool is_in_map(int cell_x, int cell_y);
};

#endif
