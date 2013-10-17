/***************************************************************************
 *  messag_handler_thread.cpp - Colli Message Handler Thread
 *
 *  Created: Thu Oct 17 16:58:00 2013
 *  Copyright  2013  AllemaniACs
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

#include "message_handler_thread.h"

#include <interfaces/MotorInterface.h>
#include <interfaces/NavigatorInterface.h>

#include <string>

using namespace fawkes;
using namespace std;

ColliMessageHandlerThread::ColliMessageHandlerThread()
  : Thread("ColliMessageHandlerThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

ColliMessageHandlerThread::~ColliMessageHandlerThread()
{
}

void
ColliMessageHandlerThread::init()
{
}


void
ColliMessageHandlerThread::finalize()
{
}

void
ColliMessageHandlerThread::loop()
{
}

