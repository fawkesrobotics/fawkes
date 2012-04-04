
/***************************************************************************
 *  blocked_timing.h - Blocked timing aspect for Fawkes
 *
 *  Created: Thu Jan 11 16:52:28 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <aspect/blocked_timing.h>
#include <core/threading/thread.h>
#include <core/exception.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BlockedTimingAspect <aspect/blocked_timing.h>
 * Thread aspect to use blocked timing.
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
 */

// Side note: Overriding Thread::run() can make our requirement useless, but
// we believe in the best of the coder: laziness

/** Constructor.
 * This special constructor is needed to define the wakeup point.
 * @param wakeup_hook hook when this thread should be woken up
 */
BlockedTimingAspect::BlockedTimingAspect(WakeupHook wakeup_hook)
{
  add_aspect("BlockedTimingAspect");
  __wakeup_hook = wakeup_hook;
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
  return __wakeup_hook;
}


/** Get string for wakeup hook.
 * @param hook wakeup hook to get string for
 * @return string representation of hook
 */
const char *
BlockedTimingAspect::blocked_timing_hook_to_string(WakeupHook hook)
{
  switch (hook) {
  case WAKEUP_HOOK_PRE_LOOP:       return "WAKEUP_HOOK_PRE_LOOP";
  case WAKEUP_HOOK_SENSOR_ACQUIRE: return "WAKEUP_HOOK_SENSOR_ACQUIRE";
  case WAKEUP_HOOK_SENSOR_PREPARE: return "WAKEUP_HOOK_SENSOR_PREPARE";
  case WAKEUP_HOOK_SENSOR_PROCESS: return "WAKEUP_HOOK_SENSOR_PROCESS";
  case WAKEUP_HOOK_WORLDSTATE:     return "WAKEUP_HOOK_WORLDSTATE";
  case WAKEUP_HOOK_THINK:          return "WAKEUP_HOOK_THINK";
  case WAKEUP_HOOK_SKILL:          return "WAKEUP_HOOK_SKILL";
  case WAKEUP_HOOK_ACT:            return "WAKEUP_HOOK_ACT";
  case WAKEUP_HOOK_ACT_EXEC:       return "WAKEUP_HOOK_ACT_EXEC";
  case WAKEUP_HOOK_POST_LOOP:      return "WAKEUP_HOOK_POST_LOOP";
  default: throw Exception("Unknown blocked timing wakeup hook");
  }
}

} // end namespace fawkes
