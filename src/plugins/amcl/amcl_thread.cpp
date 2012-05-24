
/***************************************************************************
 *  amcl_thread.cpp - Thread to perform localization
 *
 *  Created: Wed May 16 16:04:41 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include "amcl_thread.h"

using namespace fawkes;

/** @class AmclThread "amcl_thread.h"
 * Thread to perform Adaptive Monte Carlo Localization.
 * @author Tim Niemueller
 */

/** Constructor. */
AmclThread::AmclThread()
  : Thread("AmclThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

/** Destructor. */
AmclThread::~AmclThread()
{
}


void
AmclThread::init()
{
}


void
AmclThread::finalize()
{
}


void
AmclThread::loop()
{
}
