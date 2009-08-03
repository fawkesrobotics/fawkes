
/***************************************************************************
 *  wait.h - TimeWait tool
 *
 *  Created: Thu Nov 29 17:28:46 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_TIME_WAIT_H_
#define __UTILS_TIME_WAIT_H_

namespace fawkes {

class Clock;
class Time;

class TimeWait {
 public:
  TimeWait(Clock *clock, long int desired_loop_time_usec);
  ~TimeWait();

  void mark_start();
  void wait();
  void wait_systime();

  static void wait(long int usec);
  static void wait_systime(long int usec);

 private:
  Clock *__clock;
  Time  *__until;
  Time  *__until_systime;
  Time  *__now;
  long int __desired_loop_time;
};

} // end namespace fawkes

#endif
