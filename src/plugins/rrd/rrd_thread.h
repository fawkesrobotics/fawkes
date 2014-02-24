
/***************************************************************************
 *  rrd_thread.h - RRD thread
 *
 *  Created: Fri Dec 17 00:32:48 2010
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

#ifndef __PLUGINS_RRD_RRD_THREAD_H_
#define __PLUGINS_RRD_RRD_THREAD_H_

#include <plugins/rrd/aspect/rrd_manager.h>
#include <plugins/rrd/aspect/rrd_inifin.h>
#include <core/threading/thread.h>
#include <core/utils/rwlock_vector.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/aspect_provider.h>
#include <utils/time/wait.h>

class RRDThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::AspectProviderAspect,
  public fawkes::RRDManager
{
 public:
  RRDThread();
  virtual ~RRDThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  // for RRDManager
  virtual void add_rrd(fawkes::RRDDefinition *rrd_def);
  virtual void remove_rrd(fawkes::RRDDefinition *rrd_def);
  virtual void add_graph(fawkes::RRDGraphDefinition *rrd_graph_def);

  virtual void add_data(const char *rrd_name, const char *format, ...);

  virtual const fawkes::RWLockVector<fawkes::RRDDefinition *> & get_rrds() const;
  virtual const fawkes::RWLockVector<fawkes::RRDGraphDefinition *> &
    get_graphs() const;

  void generate_graphs();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::RRDAspectIniFin     __rrd_aspect_inifin;

  fawkes::RWLockVector<fawkes::RRDDefinition *>      __rrds;
  fawkes::RWLockVector<fawkes::RRDGraphDefinition *> __graphs;

  fawkes::TimeWait           *__time_wait;
  float                       __cfg_graph_interval;
};

#endif
