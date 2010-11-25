
/***************************************************************************
 *  clock.cpp - Clock aspect for Fawkes
 *
 *  Created: Tue Jun 12 22:30:33 2007
 *  Copyright  2007       Daniel Beck
 *             2007-2010  Tim Niemueller [www.niemueller.de]
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

#include <aspect/clock.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ClockAspect <aspect/clock.h>
 * Thread aspect that allows to obtain the current time from the clock.
 * Threads that need to deal with the current time should have this aspect
 * and not obtain the time by means of gettimeofday! 
 *
 * @ingroup Aspects
 * @author Daniel Beck
 */


/** @var Clock ClockAspect::clock
 * By means of this member access to the clock is given.
 */

/** Constructor. */
ClockAspect::ClockAspect()
{
  add_aspect("ClockAspect");
}

/** Virtual empty destructor. */
ClockAspect::~ClockAspect()
{
}


/** Set the clock.
 * It is guaranteed that this is called for a clock thread before
 * Thread::start() is called (when running regularly inside Fawkes).
 * @param clock Clock instance to use.
 */
void
ClockAspect::init_ClockAspect(Clock *clock)
{
  this->clock = clock;
}

} // end namespace fawkes
