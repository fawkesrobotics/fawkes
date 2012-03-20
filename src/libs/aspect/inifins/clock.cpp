
/***************************************************************************
 *  clock.cpp - Fawkes Clock Aspect initializer/finalizer
 *
 *  Created: Tue Nov 23 23:37:59 2010
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

#include <aspect/inifins/clock.h>
#include <aspect/clock.h>
#include <utils/time/clock.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ClockAspectIniFin <aspect/inifins/clock.h>
 * Initializer/finalizer for the ClockAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param clock clock instance to pass to threads
 */
ClockAspectIniFin::ClockAspectIniFin(Clock *clock)
  : AspectIniFin("ClockAspect")
{
  __clock = clock;
}


void
ClockAspectIniFin::init(Thread *thread)
{
  ClockAspect *clock_thread;
  clock_thread = dynamic_cast<ClockAspect *>(thread);
  if (clock_thread == 0) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "ClockAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  clock_thread->init_ClockAspect(__clock);
}


void
ClockAspectIniFin::finalize(Thread *thread)
{
}


} // end namespace fawkes
