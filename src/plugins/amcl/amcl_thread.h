
/***************************************************************************
 *  amcl_thread.cpp - Thread to perform localization
 *
 *  Created: Wed May 16 16:03:38 2012
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

#ifndef __PLUGINS_AMCL_AMCL_THREAD_H_
#define __PLUGINS_AMCL_AMCL_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>

#include "map/map.h"

class AmclThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlockedTimingAspect
{
 public:
  AmclThread();
  virtual ~AmclThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void read_map();

 private:
  std::string  cfg_map_file_;
  float        cfg_resolution_;
  float        cfg_origin_x_;
  float        cfg_origin_y_;
  float        cfg_origin_theta_;
  float        cfg_occupied_thresh_;
  float        cfg_free_thresh_;

  unsigned int map_width_;
  unsigned int map_height_;

  map_t       *map_;
};

#endif
