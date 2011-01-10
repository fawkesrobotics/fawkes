
/***************************************************************************
 *  rrd_example_thread.h - RRD example thread
 *
 *  Created: Mon Jan 10 00:07:36 2011
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_RRD_EXAMPLE_RRD_EXAMPLE_THREAD_H_
#define __PLUGINS_RRD_EXAMPLE_RRD_EXAMPLE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/blocked_timing.h>
#include <plugins/rrd/aspect/rrd.h>
#include <plugins/rrd/aspect/rrd_descriptions.h>

class RRDExampleThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::RRDAspect
{
 public:
  RRDExampleThread();
  virtual ~RRDExampleThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::RRDDefinition      *__test_rrd_def;
  fawkes::RRDGraphDefinition *__test_graph_def;

  unsigned int                __loop_count;
  unsigned int                __counter;
};

#endif
