
/***************************************************************************
 *  blocked_timing.h - Blocked timing aspect for Fawkes
 *
 *  Created: Thu Jan 11 16:52:28 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#include <aspect/blocked_timing.h>
#include <core/threading/thread.h>

/** @class BlockedTimingAspect aspect/blocked_timing.h
 * Thread aspect to use blocked timing
 * The Fawkes main application provides basic means to synchronize all
 * running thread with respect to several given hooks (see WakeupHook).
 * Threads of a woken up at a particular point in time. The hooks basically
 * correspond to an extended sense - plan - act kind of loop.
 * Your thread must run in Thread::OPMODE_WAITFORWAKEUP mode, otherwise it
 * is not started. This is a requirement for having the BlockedTimingAspect.
 *
 * @see Thread::OpMode
 * @ingroup Aspects
 * @author Tim Niemueller
 *
 */

// Side note: Overriding Thread::run() can make our requirement useless, but
// we believe in the best of the coder: laziness

/** Constructor.
 * This special constructor is needed to define the wakeup point.
 * @param wakeup_hook hook when this thread should be woken up
 */
BlockedTimingAspect::BlockedTimingAspect(WakeupHook wakeup_hook)
{
  this->wakeup_hook = wakeup_hook;
}


/** Virtual empty destructor. */
BlockedTimingAspect::~BlockedTimingAspect()
{
}


/** Get the wakeup hook.
 * The wakeup hook defines when this thread should be woken up. This heavily
 * depends on the used main thread.
 * @return wakeup hook
 */
BlockedTimingAspect::WakeupHook
BlockedTimingAspect::blockedTimingAspectHook() const
{
  return wakeup_hook;
}
