
/***************************************************************************
 *  blocked_timing.cpp - Fawkes Blocked Timing Aspect initializer/finalizer
 *
 *  Created: Tue Nov 23 23:25:45 2010
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

#include <aspect/inifins/blocked_timing.h>
#include <aspect/blocked_timing.h>
#include <core/macros.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BlockedTimingAspectIniFin <aspect/inifins/blocked_timing.h>
 * Initializer/finalizer for the BlockedTimingAspect.
 * @author Tim Niemueller
 */

/** Constructor. */
BlockedTimingAspectIniFin::BlockedTimingAspectIniFin()
  : AspectIniFin("BlockedTimingAspect")
{
}


void
BlockedTimingAspectIniFin::init(Thread *thread)
{
  BlockedTimingAspect *blocked_timing_thread;
  blocked_timing_thread = dynamic_cast<BlockedTimingAspect *>(thread);

  if (blocked_timing_thread == 0) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "BlockedTimingAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  if ( thread->opmode() != Thread::OPMODE_WAITFORWAKEUP) {
    throw CannotInitializeThreadException("Thread '%s' not in WAITFORWAKEUP mode"
                                          " (required for BlockedTimingAspect)",
                                          thread->name());
  }

  blocked_timing_thread->init_BlockedTimingAspect(thread);
}

void
BlockedTimingAspectIniFin::finalize(Thread *thread)
{
  BlockedTimingAspect *blocked_timing_thread;
    blocked_timing_thread = dynamic_cast<BlockedTimingAspect *>(thread);

  if (blocked_timing_thread == 0) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
      "BlockedTimingAspect, but RTTI says it "
      "has not. ", thread->name());
  }

  blocked_timing_thread->finalize_BlockedTimingAspect(thread);
}


} // end namespace fawkes
