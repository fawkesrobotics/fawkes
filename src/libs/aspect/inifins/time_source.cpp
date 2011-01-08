
/***************************************************************************
 *  timesource.cpp - Fawkes TimeSourceAspect initializer/finalizer
 *
 *  Created: Wed Nov 24 00:39:37 2010
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

#include <aspect/inifins/time_source.h>
#include <aspect/time_source.h>
#include <utils/time/clock.h>
#include <core/threading/thread_finalizer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class TimeSourceAspectIniFin <aspect/inifins/time_source.h>
 * Initializer/finalizer for the TimeSourceAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param clock clock to register time source to
 */
TimeSourceAspectIniFin::TimeSourceAspectIniFin(Clock *clock)
  : AspectIniFin("TimeSourceAspect")
{
  __clock = clock;
}


void
TimeSourceAspectIniFin::init(Thread *thread)
{
  TimeSourceAspect *timesource_thread;
  timesource_thread = dynamic_cast<TimeSourceAspect *>(thread);
  if (timesource_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "TimeSourceAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  try {
    __timesource_uc.add(timesource_thread->get_timesource());
    __clock->register_ext_timesource(timesource_thread->get_timesource(),
				     /* make default */ true);
  } catch (Exception &e) {
    throw CannotInitializeThreadException("Thread has TimeSourceAspect but there "
					  "is already another time provider.");
  }
}


void
TimeSourceAspectIniFin::finalize(Thread *thread)
{
  TimeSourceAspect *timesource_thread;
  timesource_thread = dynamic_cast<TimeSourceAspect *>(thread);
  if (timesource_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "TimeSourceAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  try {
    __clock->remove_ext_timesource(timesource_thread->get_timesource());
    __timesource_uc.remove(timesource_thread->get_timesource());
  } catch (Exception &e) {
    CannotFinalizeThreadException ce("Failed to remove time source");
    ce.append(e);
    throw;
  }
}


} // end namespace fawkes
