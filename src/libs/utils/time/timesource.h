
/***************************************************************************
 *  timesource.h - A clock's timesource
 *
 *  Created: Sun Jun 03 10:58:19 2007
 *  Copyright  2007  Daniel Beck 
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_TIME_TIMESOURCE_H_
#define __UTILS_TIME_TIMESOURCE_H_

#include <sys/time.h>

/** TimeSource interface.
 * This interfaces describes a way to interact with time sources that can be
 * given to a Clock (for instance for simulation environments).
 * @author Daniel Beck
 */
class TimeSource
{
 public:
  /** Destructor. */
  virtual ~TimeSource() {}

  /** Get the current time.
   * @param tv the current time is written to this timeval
   */
  virtual void get_time(timeval* tv) const = 0;

  /** Convert a time given w.r.t. this time sources into system time.
   * @param tv the time to convert
   * @return the converted time
   */
  virtual timeval conv_to_realtime(const timeval* tv) const = 0;
};

#endif /*  __UTILS_TIME_TIMESOURCE_H_ */
