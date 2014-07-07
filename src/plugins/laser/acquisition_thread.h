
/***************************************************************************
 *  acquisition_thread.h - Thread that retrieves the laser data
 *
 *  Created: Wed Oct 08 13:41:02 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LASER_ACQUISITION_THREAD_H_
#define __PLUGINS_LASER_ACQUISITION_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>

namespace fawkes {
  class Mutex;
  class Configuration;
  class Logger;
  class Time;
}

class LaserAcquisitionThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect
{
 public:
  LaserAcquisitionThread(const char *thread_name);
  virtual ~LaserAcquisitionThread();

  bool lock_if_new_data();
  void unlock();

  virtual void   pre_init(fawkes::Configuration *config,
			  fawkes::Logger *logger) = 0;

  const float *  get_distance_data();
  const float *  get_echo_data();
  const fawkes::Time *   get_timestamp();

  unsigned int   get_distance_data_size();
  unsigned int   get_echo_data_size();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 protected:
  void alloc_distances(unsigned int num_distances);
  void alloc_echoes(unsigned int num_echoes);
  void reset_distances();
  void reset_echoes();

 protected:
  fawkes::Mutex    *_data_mutex;
  fawkes::Time     *_timestamp;

  bool    _new_data;
  float  *_distances;
  float  *_echoes;

  unsigned int  _distances_size;
  unsigned int  _echoes_size;

};


#endif
