
/***************************************************************************
 *  manager.h - Fawkes Aspect Manager
 *
 *  Created: Thu Nov 25 00:27:42 2010 (based on inifin.h)
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __ASPECT_MANAGER_H_
#define __ASPECT_MANAGER_H_

#include <core/threading/thread_initializer.h>
#include <core/threading/thread_finalizer.h>

#include <map>
#include <list>
#include <string>

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
class LoggerEmployer;
class BlockedTimingExecutor;
class MainLoopEmployer;
class AspectIniFin;
class SyncPointManager;

namespace tf {
  class Transformer;
}

class AspectManager : public ThreadInitializer, public ThreadFinalizer
{
 public:
  virtual ~AspectManager();

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);
  virtual bool prepare_finalize(Thread *thread);

  void register_inifin(AspectIniFin *inifin);
  void unregister_inifin(AspectIniFin *inifin);

  bool has_threads_for_aspect(const char *aspect_name);

  void register_default_inifins(BlackBoard *blackboard,
				ThreadCollector *collector,
				Configuration *config,
				Logger *logger,
				Clock *clock,
				FawkesNetworkHub *fnethub,
				MainLoopEmployer *mloop_employer,
				LoggerEmployer *logger_employer,
				BlockedTimingExecutor *btexec,
				NetworkNameResolver *nnresolver,
				ServicePublisher *service_publisher,
				ServiceBrowser *service_browser,
				PluginManager *pmanager,
				tf::Transformer *tf_listener,
				SyncPointManager *syncpoint_manager);

 private:
  std::map<std::string, AspectIniFin *> __inifins;
  std::map<std::string, AspectIniFin *> __default_inifins;
  std::map<std::string, std::list<Thread *> > __threads;
};


} // end namespace fawkes

#endif
