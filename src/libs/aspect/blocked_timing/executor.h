
/***************************************************************************
 *  blocked_timing_runner.h - Interface to exec BlockedTimingAspect threads
 *
 *  Created: Sat Aug 02 11:45:59 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __ASPECT_BLOCKED_TIMING_EXECUTOR_H_
#define __ASPECT_BLOCKED_TIMING_EXECUTOR_H_

#include <aspect/blocked_timing.h>
#include <list>
#include <string>

namespace fawkes {

class Barrier;

class BlockedTimingExecutor
{
 public:
  virtual ~BlockedTimingExecutor();

  virtual void wakeup_and_wait(BlockedTimingAspect::WakeupHook hook,
			       unsigned int timeout_usec = 0)         = 0;
  virtual void wakeup(BlockedTimingAspect::WakeupHook hook,
		      Barrier *barrier = 0)                           = 0;

  virtual void try_recover(std::list<std::string> &recovered_threads) = 0;

  virtual bool timed_threads_exist()                                  = 0;
  virtual void wait_for_timed_threads()                               = 0;
  virtual void interrupt_timed_thread_wait()                          = 0;
};

} // end namespace fawkes

#endif
