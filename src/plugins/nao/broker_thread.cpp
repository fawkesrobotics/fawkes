
/***************************************************************************
 *  broker_thread.cpp - NaoQi broker providing Thread
 *
 *  Created: Thu May 12 19:03:40 2011
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

#include "broker_thread.h"
#include "naoqi_broker.h"

#include <cerrno>
#include <csignal>
#include <unistd.h>
#include <sys/wait.h>

#include <alcommon/albrokermanager.h>
#include <alcore/alerror.h>

using namespace fawkes;

/** @class NaoQiBrokerThread "broker_thread.h"
 * NaoQi Broker Thread.
 * This thread maintains a NaoQi broker which can be used by other
 * threads and is provided via the NaoQiAspect.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
NaoQiBrokerThread::NaoQiBrokerThread()
  : Thread("NaoQiBrokerThread", Thread::OPMODE_WAITFORWAKEUP),
    AspectProviderAspect("NaoQiAspect", &__naoqi_aspect_inifin)
{
}


/** Destructor. */
NaoQiBrokerThread::~NaoQiBrokerThread()
{
}


void
NaoQiBrokerThread::init()
{
  if (fawkes::naoqi::broker) {
    __broker = fawkes::naoqi::broker;
    logger->log_debug(name(), "Using NaoQi Module broker");
  } else {
    throw Exception("NaoQi broker not set, embedding of NaoQi "
		    "not implemented, yet");
  }

  /*
  std::vector<AL::ALModuleInfo>::iterator m;
  AL::ALPtr<std::vector<AL::ALModuleInfo> >
    modules(new std::vector<AL::ALModuleInfo>);

  __broker->getModuleList(modules);
  for (m = modules->begin(); m != modules->end(); ++m) {
    printf("Module: %s @ %s:%i\n", m->name.c_str(), m->ip.c_str(), m->port);
  }

  modules->clear();
  __broker->getGlobalModuleList(modules);
  for (m = modules->begin(); m != modules->end(); ++m) {
    printf("Global Module: %s @ %s:%i\n", m->name.c_str(), m->ip.c_str(), m->port);
  }
  */

  __naoqi_aspect_inifin.set_naoqi_broker(__broker);
}


void
NaoQiBrokerThread::finalize()
{
  __broker.reset();
  __naoqi_aspect_inifin.set_naoqi_broker(__broker);
}


void
NaoQiBrokerThread::loop()
{
}
