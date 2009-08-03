
/***************************************************************************
 *  inifin.h - Fawkes Aspect initializer/finalizer
 *
 *  Created: Tue Jan 30 13:34:54 2007
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __ASPECT_INIFIN_H_
#define __ASPECT_INIFIN_H_

#include <core/threading/thread_initializer.h>
#include <core/threading/thread_finalizer.h>
#include <core/threading/thread_notification_listener.h>

#include <utils/constraints/dependency_onetomany.h>
#include <utils/constraints/unique.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class BlackBoard;
class Configuration;
class Logger;
class Clock;
class FawkesNetworkHub;
class PluginManager;
class Thread;
class ThreadCollector;
class NetworkNameResolver;
class ServicePublisher;
class ServiceBrowser;
class TimeSource;
class LoggerEmployer;
class BlockedTimingExecutor;
class MainLoopEmployer;
class MainLoopAspect;
#ifdef HAVE_FIREVISION
class VisionMasterAspect;
class VisionAspect;
#endif

class AspectIniFin
: public ThreadInitializer,
  public ThreadFinalizer,
  public ThreadNotificationListener
{
 public:
  AspectIniFin(BlackBoard *blackboard, ThreadCollector *collector,
	       Configuration *config, Logger *logger, Clock *clock);
  virtual ~AspectIniFin();

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);
  virtual bool prepare_finalize(Thread *thread);

  void set_fnet_hub(FawkesNetworkHub *fnethub);
  void set_mainloop_employer(MainLoopEmployer *employer);
  void set_logger_employer(LoggerEmployer *employer);
  void set_blocked_timing_executor(BlockedTimingExecutor *btexec);
  void set_network_members(NetworkNameResolver *nnresolver,
			   ServicePublisher *service_publisher,
			   ServiceBrowser *service_browser);
  void set_plugin_manager(PluginManager *manager);

  virtual bool thread_started(Thread *thread) throw();
  virtual bool thread_init_failed(Thread *thread) throw();

 private:
  BlackBoard            *__blackboard;
  ThreadCollector       *__thread_collector;
  Configuration         *__config;
  Logger                *__logger;
  Clock                 *__clock;
  FawkesNetworkHub      *__fnethub;
  NetworkNameResolver   *__nnresolver;
  ServicePublisher      *__service_publisher;
  ServiceBrowser        *__service_browser;
  MainLoopEmployer      *__mainloop_employer;
  BlockedTimingExecutor *__btexec;
  LoggerEmployer        *__logger_employer;
  PluginManager         *__plugin_manager;

#ifdef HAVE_FIREVISION
  OneToManyDependency<VisionMasterAspect, VisionAspect> __vision_dependency;
#endif
  UniquenessConstraint<TimeSource>     __timesource_uc;
  UniquenessConstraint<MainLoopAspect> __mainloop_uc;
};

} // end namespace fawkes

#endif
