
/***************************************************************************
 *  blocked_timing.h - Blocked timing aspect for Fawkes
 *
 *  Created: Thu Jan 11 16:49:25 2007
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

#ifndef __ASPECT_BLOCKED_TIMING_H_
#define __ASPECT_BLOCKED_TIMING_H_

#include <aspect/aspect.h>
#include <aspect/syncpoint.h>
#include <core/threading/thread_loop_listener.h>

#include <map>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class BlockedTimingLoopListener
 * Loop Listener of the BlockedTimingAspect.
 * This loop listener immediately wakes up the thread after loop returned.
 * The thread will then wait for the syncpoint of the next iteration.
 * The BlockedTimingAspect cannot be derived from ThreadLoopListener because
 * the SyncPointAspect is already derived from ThreadLoopListener and we need
 * another listener. Therefore, use composition instead.
 */
class BlockedTimingLoopListener : public ThreadLoopListener
{
 public:
  void post_loop(Thread *thread);
};

class BlockedTimingAspect : public SyncPointAspect
{
 public:
  /** Type to define at which hook the thread is woken up.
   * See FawkesMainThread for information when and in which order the hooks
   * are called.
   * @see FawkesMainThread::loop()
   */
  typedef enum {
    WAKEUP_HOOK_PRE_LOOP,	/**< before each loop */
    WAKEUP_HOOK_SENSOR_ACQUIRE,	/**< sensor acquisition thread,
                                 *  acquire data from sensor */
    WAKEUP_HOOK_SENSOR_PREPARE,	/**< sensor data preparation thread,
                                 * convert acquired data to usable format */
    WAKEUP_HOOK_SENSOR_PROCESS,	/**< sensor data processing thread */
    WAKEUP_HOOK_WORLDSTATE,	/**< world state thread */
    WAKEUP_HOOK_THINK,		/**< think thread (agent) */
    WAKEUP_HOOK_SKILL,		/**< skill thread (skill module) */
    WAKEUP_HOOK_ACT,		/**< act thread (motor module etc.) */
    WAKEUP_HOOK_ACT_EXEC,	/**< act execution thread */
    WAKEUP_HOOK_POST_LOOP	/**< run after loop */
  } WakeupHook;

  BlockedTimingAspect(WakeupHook wakeup_hook);
  virtual ~BlockedTimingAspect();

  static const char *  blocked_timing_hook_to_string(WakeupHook hook);

  static std::string blocked_timing_hook_to_start_syncpoint(WakeupHook hook);
  static std::string blocked_timing_hook_to_end_syncpoint(WakeupHook hook);

  void init_BlockedTimingAspect(Thread *thread);
  void finalize_BlockedTimingAspect(Thread *thread);

  WakeupHook blockedTimingAspectHook() const;

  /** Translation from WakeupHooks to SyncPoints. Each WakeupHook corresponds to
   *  exactly one SyncPoint, e.g., WAKEUP_HOOK_PRE_LOOP becomes /preloop.
   */
  static const std::map<const WakeupHook, const std::string> hook_to_syncpoint;

 private:
  WakeupHook __wakeup_hook;
  BlockedTimingLoopListener *__loop_listener;
};

} // end namespace fawkes

#endif
