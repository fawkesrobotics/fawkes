
/***************************************************************************
 *  map_filter.cpp - Laser map data filter
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

#include "map_filter.h"

#include <core/exception.h>
#include <utils/time/time.h>
#include <utils/math/coord.h>
#include <cmath>
#include <string>
#include <limits>

/** @class LaserMapFilterDataFilter "map_filter.h
 * Removes static laser data (laser beams near occupied map cells)
 * @author Tobias Neumann
 */

/** Constructor.
 * @param filter_name name of this filter
 * @param in_data_size number of entries input value arrays
 * @param in vector of input arrays
 * @param tf_listener to access the tf::Transformer aspect
 * @param config to access the Configuration aspect
 * @param logger to access the Logger aspect
 */
LaserMapFilterDataFilter::LaserMapFilterDataFilter(const std::string filter_name,
                                                   unsigned int in_data_size,
                                                   std::vector<LaserDataFilter::Buffer *> &in,
                                                   fawkes::tf::Transformer *tf_listener,
                                                   fawkes::Configuration *config,
                                                   fawkes::Logger *logger)
	: LaserDataFilter(filter_name, in_data_size, in, 1)
{
  tf_listener_ = tf_listener;
  config_ = config;
  logger_ = logger;
  map_ = load_map();
  frame_map_ = config_->get_string("/frames/fixed");
  cfg_occupied_thresh_ = std::numeric_limits<float>::max();
}

/** loads map using amcl
 * @return the loaded map
 */
map_t *
LaserMapFilterDataFilter::load_map()
{
  std::vector<std::pair<int, int>> free_space_indices;
  std::string  cfg_map_file;
  float        cfg_resolution;
  float        cfg_origin_x;
  float        cfg_origin_y;
  float        cfg_origin_theta;
  float        cfg_free_thresh;

  fawkes::amcl::read_map_config(config_, cfg_map_file, cfg_resolution, cfg_origin_x,
        cfg_origin_y, cfg_origin_theta, cfg_occupied_thresh_,
        cfg_free_thresh);

  return fawkes::amcl::read_map(cfg_map_file.c_str(),
        cfg_origin_x, cfg_origin_y, cfg_resolution,
        cfg_occupied_thresh_, cfg_free_thresh, free_space_indices);
}

/** Returnes whenever a given cell is within the map or not
 *  @param cell_x the x position of the cell
 *  @param cell_y the y position of the cell
 *  @return true if the cell is within the map
 *          false otherwise
 */
bool
LaserMapFilterDataFilter::is_in_map(int cell_x, int cell_y)
{
  if (cell_x < 0 || cell_x > map_->size_x ||
      cell_y < 0 || cell_y > map_->size_y) {
    return false;
  }
  return true;
}

void
LaserMapFilterDataFilter::filter()
{
  const unsigned int vecsize = in.size();
  if (vecsize == 0)  return;

  for (unsigned int a = 0; a < vecsize; ++a) {
    // get tf to map of laser input
    fawkes::tf::StampedTransform transform;
    try{
      tf_listener_->lookup_transform(frame_map_.c_str(), in[a]->frame, *(in[a]->timestamp), transform);
    } catch(fawkes::tf::TransformException &e) {
      try{
        tf_listener_->lookup_transform(frame_map_.c_str(), in[a]->frame, fawkes::Time(0, 0), transform);
        //logger_->log_debug("map_filter", "Can't transform laser-data using newest tf\n(%s\t%s\t\%lf)",
        //    frame_map_.c_str(), in[a]->frame.c_str(), in[a]->timestamp->in_sec());
      } catch(fawkes::tf::TransformException &e) {
        logger_->log_warn("map_filter", "Can't transform laser-data (%s -> %s)",
                          frame_map_.c_str(), in[a]->frame.c_str());
        return;
      }
    }
    // set out meta info
    out[a]->frame = in[a]->frame;
    out[a]->timestamp = in[a]->timestamp;
    // for each point
    for (unsigned int i = 0; i < out_data_size; ++i) {
      bool add = true;
      // check nan
      if ( std::isfinite(in[a]->values[i]) ) {
        // transform to cartesian
        double angle = M_PI * (360.f / out_data_size * i ) / 180;

        float x, y;
        fawkes::polar2cart2d(angle, in[a]->values[i], &x, &y);

        // transform into map
        fawkes::tf::Point p;
        p.setValue(x, y, 0.);
        p = transform * p;

        // transform to map cells
        int cell_x = (int)MAP_GXWX(map_, p.getX());
        int cell_y = (int)MAP_GYWY(map_, p.getY());

        // search in 8-neighborhood and itself for occupied pixels in map
        for (int ox = -2; add && ox <= 2; ++ox) {
          for (int oy = -2; oy <= 2; ++oy) {
            int x = cell_x + ox;
            int y = cell_y + oy;
            if (MAP_VALID(map_, x, y)) {
              if (map_->cells[MAP_INDEX(map_, x, y)].occ_state > 0) {
                add = false;
                break;
              }
            }
          }
        }
      }
      if (add) {
        out[a]->values[i] = in[a]->values[i];
      } else {
        out[a]->values[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
}
