
/***************************************************************************
 *  inifin.h - Fawkes Aspect initializer/finalizer
 *
 *  Created: Tue Jan 30 13:34:54 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __ASPECT_INIFIN_H_
#define __ASPECT_INIFIN_H_

#include <core/threading/thread_initializer.h>
#include <core/threading/thread_finalizer.h>
#include <core/threading/thread_notification_listener.h>

class BlackBoard;
class Configuration;
class Logger;
class Clock;
class FawkesNetworkHub;
class Thread;
class ThreadCollector;
class NetworkNameResolver;
class ServicePublisher;
class ServiceBrowser;
template <class Provider, class Dependant>
  class OneToManyDependency;
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
  void set_network_members(NetworkNameResolver *nnresolver,
			   ServicePublisher *service_publisher,
			   ServiceBrowser *service_browser);

  virtual void thread_started(Thread *thread);
  virtual void thread_init_failed(Thread *thread);

 private:
  BlackBoard          *__blackboard;
  ThreadCollector     *__thread_collector;
  Configuration       *__config;
  Logger              *__logger;
  Clock               *__clock;
  FawkesNetworkHub    *__fnethub;
  NetworkNameResolver *__nnresolver;
  ServicePublisher    *__service_publisher;
  ServiceBrowser      *__service_browser;

#ifdef HAVE_FIREVISION
  OneToManyDependency<VisionMasterAspect, VisionAspect> *__vision_dependency;
#endif
};


#endif
