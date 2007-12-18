
/***************************************************************************
 *  wait.h - TimeWait tool
 *
 *  Created: Thu Nov 29 17:28:46 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_TIME_WAIT_H_
#define __UTILS_TIME_WAIT_H_

class Clock;
class Time;

class TimeWait {
 public:
  TimeWait(Clock *clock, long int desired_loop_time);
  ~TimeWait();

  void mark_start();
  void wait();

  static void wait(long int usec);
  static void wait_systime(long int usec);

 private:
  Clock *__clock;
  Time  *__until;
  Time  *__now;
  long int __desired_loop_time;
};

#endif
