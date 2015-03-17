/***************************************************************************
 *  amcl_utils.h - AMCL utils
 *
 *  Created: Thu Aug 23 18:10:03 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_AMCL_AMCL_UTILS_H_
#define __PLUGINS_AMCL_AMCL_UTILS_H_

#include "map/map.h"

#include <vector>
#include <string>

#define AMCL_CFG_PREFIX "/plugins/amcl/"

namespace fawkes {

  class Configuration;

  namespace amcl {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

map_t *
read_map(const char *map_file,
	 float origin_x, float origin_y, float resolution,
	 float occupied_threshold, float free_threshold,
	 std::vector<std::pair<int, int> > &free_space_indices);

void
read_map_config(Configuration *config,
		std::string  &cfg_map_file, float &cfg_resolution,
		float &cfg_origin_x, float &cfg_origin_y, float &cfg_origin_theta,
		float &cfg_occupied_thresh, float &cfg_free_thresh,
		std::string cfg_prefix = AMCL_CFG_PREFIX);


} // end namespace amcl
} // end namespace fawkes

#endif
