
/***************************************************************************
 *  context_thread.cpp - OpenNI context providing Thread
 *
 *  Created: Sat Feb 26 17:46:29 2011
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

#include "context_thread.h"

#include <XnCppWrapper.h>

using namespace fawkes;

/** @class OpenNiContextThread "context_thread.h"
 * OpenNI Context Thread.
 * This thread maintains an OpenNI context which can be used by other
 * threads and is provided via the OpenNiAspect.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
OpenNiContextThread::OpenNiContextThread()
  : Thread("OpenNiContextThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR),
    AspectProviderAspect("OpenNiAspect", &__openni_aspect_inifin)
{
  set_prepfin_conc_loop(true);
}


/** Destructor. */
OpenNiContextThread::~OpenNiContextThread()
{
}


void
OpenNiContextThread::init()
{
  __openni = new xn::Context();

  XnStatus st;
  if ((st = __openni->Init()) != XN_STATUS_OK) {
    __openni.clear();
    throw Exception("Initializing OpenNI failed: %s", xnGetStatusString(st));
  }

  __openni_aspect_inifin.set_openni_context(__openni);
}


void
OpenNiContextThread::finalize()
{
  __openni->StopGeneratingAll();
  __openni->Shutdown();
  __openni.clear();
  __openni_aspect_inifin.set_openni_context(__openni);
}


void
OpenNiContextThread::loop()
{
  __openni.lock();
  __openni->WaitNoneUpdateAll();
  __openni.unlock();
}
