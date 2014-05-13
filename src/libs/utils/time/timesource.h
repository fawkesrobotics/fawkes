
/***************************************************************************
 *  timesource.h - A clock's timesource
 *
 *  Created: Sun Jun 03 10:58:19 2007
 *  Copyright  2007  Daniel Beck 
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

#ifndef __UTILS_TIME_TIMESOURCE_H_
#define __UTILS_TIME_TIMESOURCE_H_

#include <sys/time.h>

namespace fawkes {

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

  /** Convert a native time to the external time.
   * When communicating with another instance which provides times in
   * some timeformat native to the underlying time source (e.g. received
   * from a simulation) it must be converted to a Fawkes time.
   * @param tv time in external time source native format
   * @return time in Fawkes comparable to other times generated using
   * the external timesource.
   */
  virtual timeval conv_native_to_exttime(const timeval* tv) const = 0;
};

} // end namespace fawkes

#endif /*  __UTILS_TIME_TIMESOURCE_H_ */
