
/***************************************************************************
 *  static_transform_thread.h - Static transform publisher thread
 *
 *  Created: Tue Oct 25 16:32:04 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_STATIC_TRANSFORMS_STATIC_TRANSFORMS_THREAD_H_
#define __PLUGINS_STATIC_TRANSFORMS_STATIC_TRANSFORMS_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/tf.h>

namespace fawkes {
  class Time;
}

class StaticTransformsThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::TransformAspect
{
 public:
  StaticTransformsThread();
  virtual ~StaticTransformsThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  /** Static transform entry to publish. */
  typedef struct {
    std::string name;	/**< Transform name */
    fawkes::tf::StampedTransform *transform;	/**< Transform. */
  } Entry;

  std::list<Entry> __entries;

  float __cfg_update_interval;
  fawkes::Time *__last_update;
};

#endif
