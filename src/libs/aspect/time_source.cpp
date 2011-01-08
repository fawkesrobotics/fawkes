
/***************************************************************************
 *  time_source.cpp - Time source aspect for Fawkes
 *
 *  Created: Sun Feb 24 13:34:37 2008
 *  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
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

#include <aspect/aspect.h>
#include <aspect/time_source.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class TimeSourceAspect <aspect/time_source.h>
 * Thread aspect that allows to provide a time source to the Fawkes clock.
 * There may be at most one external time source provided by a thread with
 * the TimeSourceAspect at any given time. This is ensured by aspect
 * initializer.
 * This aspect can be used for example to attach Fawkes to a simulator and
 * provide the simulated time to the system.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** Constructor.
 * @param timesource the time source to provide to Fawkes
 */
TimeSourceAspect::TimeSourceAspect(TimeSource *timesource)
{
  add_aspect("TimeSourceAspect");
  __time_source = timesource;
}

/** Virtual empty destructor. */
TimeSourceAspect::~TimeSourceAspect()
{
}


/** Get time source.
 * This method is called by the aspect initializer to get the time source
 * the thread with this aspect provides.
 * @return time source provided by the thread with this aspect
 */
TimeSource *
TimeSourceAspect::get_timesource() const
{
  return __time_source;
}

} // end namespace fawkes
