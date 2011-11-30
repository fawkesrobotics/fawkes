
/***************************************************************************
 *  context_thread.h - OpenNI context providing thread
 *
 *  Created: Sat Feb 26 15:23:16 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_OPENNI_CONTEXT_THREAD_H_
#define __PLUGINS_OPENNI_CONTEXT_THREAD_H_

#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blocked_timing.h>
#include <aspect/aspect_provider.h>
#include <plugins/openni/aspect/openni_inifin.h>
#include <utils/time/time.h>

#include <sys/types.h>
#include <map>
#include <string>

namespace xn {
  class Context;
  class Device;
}

class OpenNiContextThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::AspectProviderAspect
{
 public:
  OpenNiContextThread();
  virtual ~OpenNiContextThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  void print_nodes();
  void verify_active();
  void start_sensor_server();
  void stop_sensor_server();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::LockPtr<xn::Context>  __openni;
  fawkes::OpenNiAspectIniFin    __openni_aspect_inifin;

  bool         __cfg_run_sensor_server;
  std::string  __cfg_sensor_bin;
  pid_t        __sensor_server_pid;
  xn::Device  *__device;

  int __last_refcount;

  fawkes::Time __check_last;
  fawkes::Time __check_now;

  unsigned int __device_no_data_loops;

  std::map<std::string, unsigned int> __dead_loops;
};

#endif
