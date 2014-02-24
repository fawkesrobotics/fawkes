/***************************************************************************
 *  amcl_utils.cpp - AMCL utils
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

#include "amcl_utils.h"
#include <fvutils/readers/png.h>
#include <config/config.h>
#include <cstdlib>

using namespace firevision;

namespace fawkes {
  namespace amcl {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

/** Read map.
 * @param map_file filename of map
 * @param origin_x origin x offset
 * @param origin_y origin y offset
 * @param resolution map resolution
 * @param occupied_threshold minimum threshold when to consider a cell occupied
 * @param free_threshold maximum threshold when to consider a cell free
 * @param free_space_indices upon return contains indices of free cells
 * @return loaded map
 */
map_t *
read_map(const char *map_file,
	 float origin_x, float origin_y, float resolution,
	 float occupied_threshold, float free_threshold,
	 std::vector<std::pair<int, int> > &free_space_indices)
{
  map_t *map;

  firevision::PNGReader png_reader(map_file);
  unsigned int map_width  = png_reader.pixel_width();
  unsigned int map_height = png_reader.pixel_height();
  unsigned char *img_buffer = malloc_buffer(firevision::YUV422_PLANAR,
					    map_width, map_height);
  png_reader.set_buffer(img_buffer);
  png_reader.read();

  map = map_alloc();
  map->size_x   = map_width;
  map->size_y   = map_height;
  map->scale    = resolution;
  map->origin_x = origin_x + (map->size_x / 2) * map->scale;
  map->origin_y = origin_y + (map->size_y / 2) * map->scale;
  map->cells =
    (map_cell_t*) malloc(sizeof(map_cell_t) * map->size_x * map->size_y);

  for (unsigned int h = 0; h < map_height; ++h) {
    for (unsigned int w = 0; w < map_width; ++w) {
      unsigned int i = h * map_width + w;
      float y = (255 - img_buffer[i]) / 255.;

      // Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.

      if (y > occupied_threshold) {
	map->cells[MAP_IDX(map_width, w, map_height - h - 1)].occ_state = +1;
      } else if (y <= free_threshold) {
	map->cells[MAP_IDX(map_width, w, map_height - h - 1)].occ_state = -1;
	free_space_indices.push_back(std::make_pair(w,map_height - h - 1));
      } else {
	map->cells[MAP_IDX(map_width, w, map_height - h - 1)].occ_state =  0;
      }
    }
  }
  free(img_buffer);

  return map;
}


/** Read map configuration.
 * @param config configuration to read from
 * @param cfg_map_file upon returns contains map filename
 * @param cfg_resolution upon return contains map resolution
 * @param cfg_origin_x upon return contains origin x offset
 * @param cfg_origin_y upon return contains origin y offset
 * @param cfg_occupied_thresh upon return contains minimum threshold
 * when to consider a cell occupied
 * @param cfg_free_thresh upon return contains maximum threshold when
 * to consider a cell free
 * @param cfg_prefix optional config path prefix
 */
void
read_map_config(Configuration *config,
		std::string  &cfg_map_file, float &cfg_resolution,
		float &cfg_origin_x, float &cfg_origin_y, float &cfg_origin_theta,
		float &cfg_occupied_thresh, float &cfg_free_thresh,
		std::string cfg_prefix)
{
  cfg_map_file =
    std::string(CONFDIR) + "/" + config->get_string((cfg_prefix + "map_file").c_str());
  cfg_resolution = config->get_float((cfg_prefix + "resolution").c_str());
  cfg_origin_x = config->get_float((cfg_prefix + "origin_x").c_str());
  cfg_origin_y = config->get_float((cfg_prefix + "origin_y").c_str());
  cfg_origin_theta = config->get_float((cfg_prefix + "origin_theta").c_str());
  cfg_occupied_thresh = config->get_float((cfg_prefix + "occupied_threshold").c_str());
  cfg_free_thresh = config->get_float((cfg_prefix + "free_threshold").c_str());
}


} // end namespace amcl
} // end namespace fawkes
