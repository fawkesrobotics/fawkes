
/***************************************************************************
 *  broker_thread.h - NaoQi broker providing thread
 *
 *  Created: Thu May 12 19:01:52 2011
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

#ifndef __PLUGINS_NAO_BROKER_THREAD_H_
#define __PLUGINS_NAO_BROKER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/aspect_provider.h>
#include <plugins/nao/aspect/naoqi_inifin.h>

#include <alcore/alptr.h>
#include <alcommon/albroker.h>

class NaoQiBrokerThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::AspectProviderAspect
{
 public:
  NaoQiBrokerThread();
  virtual ~NaoQiBrokerThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  AL::ALPtr<AL::ALBroker>      __broker;
  fawkes::NaoQiAspectIniFin    __naoqi_aspect_inifin;

};

#endif
