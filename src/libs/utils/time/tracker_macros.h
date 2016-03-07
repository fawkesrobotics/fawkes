
/***************************************************************************
 *  tracker_macros.h - Time tracker convenience macros
 *
 *  Created: Fri Nov 30 15:00:13 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_TIME_TRACKER_MACROS_H_
#define __UTILS_TIME_TRACKER_MACROS_H_

#ifndef TRACKER_VARIABLE
#  define TRACKER_VARIABLE tt_
#endif

#ifdef USE_TIMETRACKER
#  define TIMETRACK_START(c)			\
  TRACKER_VARIABLE->ping_start(c);		\
  
#  define TIMETRACK_INTER(c1, c2)		\
  TRACKER_VARIABLE->ping_end(c1);		\
  TRACKER_VARIABLE->ping_start(c2);

#  define TIMETRACK_END(c)			\
  TRACKER_VARIABLE->ping_end(c);

#  define TIMETRACK_ABORT(c)			\
  TRACKER_VARIABLE->ping_abort(c);

#  define TIMETRACK_SCOPE(c)                    \
  fawkes::ScopedClassItemTracker __tt_scope_sentry(*TRACKER_VARIABLE, c);

#else
#  define TIMETRACK_START(c)
#  define TIMETRACK_INTER(c1, c2)
#  define TIMETRACK_END(c)
#  define TIMETRACK_ABORT(c)
#  define TIMETRACK_SCOPE(c)
#endif


#endif
