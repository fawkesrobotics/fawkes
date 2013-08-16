
/***************************************************************************
 *  cedar_thread.cpp -  CLIPS-based agent main thread
 *
 *  Created: Fri Aug 16 18:00:32 2013 +0200
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include "cedar_thread.h"

#include <core/threading/mutex_locker.h>

using namespace fawkes;

/** @class CedarThread "cedar_thread.h"
 * Main thread of CEDAR error analysis plugin.
 * @author Tim Niemueller
 */

/** Constructor. */
CedarThread::CedarThread()
  : Thread("CedarThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK),
    CLIPSAspect("cedar", "CEDAR")
{
}


/** Destructor. */
CedarThread::~CedarThread()
{
}


void
CedarThread::init()
{
  clips->assert_fact("(cedar-init)");
  clips->refresh_agenda();
  clips->run();
}


void
CedarThread::finalize()
{
}


void
CedarThread::loop()
{
  MutexLocker lock(clips.objmutex_ptr());
}


