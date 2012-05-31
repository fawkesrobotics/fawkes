
/***************************************************************************
 *  amcl_thread.cpp - Thread to perform localization
 *
 *  Created: Wed May 16 16:04:41 2012
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

#include "amcl_thread.h"

#include <fvutils/readers/png.h>
#include <cstdlib>

using namespace fawkes;

/** @class AmclThread "amcl_thread.h"
 * Thread to perform Adaptive Monte Carlo Localization.
 * @author Tim Niemueller
 */

/** Constructor. */
AmclThread::AmclThread()
  : Thread("AmclThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
  map_ = NULL;
}

/** Destructor. */
AmclThread::~AmclThread()
{
}


void
AmclThread::init()
{
  std::string cfg_prefix = "/plugins/amcl/";

  cfg_map_file_        =
    std::string(CONFDIR) + config->get_string((cfg_prefix + "map_file").c_str());
  cfg_resolution_      = config->get_float((cfg_prefix + "resolution").c_str());
  cfg_origin_x_        = config->get_float((cfg_prefix + "origin_x").c_str());
  cfg_origin_y_        = config->get_float((cfg_prefix + "origin_y").c_str());
  cfg_origin_theta_    = config->get_float((cfg_prefix + "origin_theta").c_str());
  cfg_occupied_thresh_ =
    config->get_float((cfg_prefix + "occupied_threshold").c_str());
  cfg_free_thresh_     =
    config->get_float((cfg_prefix + "free_threshold").c_str());

  read_map();
}


void
AmclThread::read_map()
{
  firevision::PNGReader png_reader(cfg_map_file_.c_str());
  map_width_  = png_reader.pixel_width();
  map_height_ = png_reader.pixel_height();
  unsigned char *img_buffer =
    malloc_buffer(firevision::YUV422_PLANAR, map_width_, map_height_);
  png_reader.set_buffer(img_buffer);
  png_reader.read();

  map_ = map_alloc();
  map_->size_x = map_width_;
  map_->size_y = map_height_;
  map_->scale   = cfg_resolution_;
  map_->origin_x = cfg_origin_x_ + (map_->size_x / 2) * map_->scale;
  map_->origin_y = cfg_origin_y_ + (map_->size_y / 2) * map_->scale;
  map_->cells =
    (map_cell_t*)malloc(sizeof(map_cell_t) * map_->size_x*map_->size_y);

  for (unsigned int h = 0; h < map_height_; ++h) {
    for (unsigned int w = 0; w < map_width_; ++w) {
      unsigned int i = h * map_width_ + w;
      float y = img_buffer[i] / 255.;

      if (y > cfg_occupied_thresh_) {
        map_->cells[i].occ_state = +1;
      } else if (y < cfg_free_thresh_) {
        map_->cells[i].occ_state = 0;
      } else {
        map_->cells[i].occ_state = -1;
      }
    }
  }
  free(img_buffer);
}

void
AmclThread::finalize()
{
  if (map_) {
    map_free(map_);
    map_ = NULL;
  }
}


void
AmclThread::loop()
{
}
