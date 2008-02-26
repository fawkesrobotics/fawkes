
/***************************************************************************
 *  time.h - Time utils
 *
 *  Created: Wed Jan 18 15:56:33 2006 (from FireVision)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_TIME_TIME_H_
#define __UTILS_TIME_TIME_H_

#include <sys/time.h>

/** Calculate time difference of two time structs.
 * The calculated time is t = a - b, where t is a represented as the number of
 * seconds in a single precision float.
 * @param a time to subtract from
 * @param b time to subtract
 * @return a - b
 */
inline float
time_diff_sec(const timeval &a, const timeval &b)
{
  return a.tv_sec  - b.tv_sec + (a.tv_usec - b.tv_usec) / 1000000.f;
}


/** Calculate time difference of two time structs.
 * The calculated time is t = a - b, where t is a represented as the number of
 * seconds in a single precision float.
 * @param a_sec seconds of time to subtract from
 * @param a_usec microseconds of time to subtract from
 * @param b_sec seconds of time to subtract
 * @param b_usec microseconds of time to subtract
 * @return a_sec - b_sec  + (a_usec - b_usec) / 1000000.f
 */
inline float
time_diff_sec(const long int a_sec, const long int a_usec,
	      const long int b_sec, const long int b_usec)
{
  return a_sec - b_sec + (a_usec - b_usec) / 1000000.f;
}


/** Get difference between two time structs in microseconds.
 * The calculated time is t = a - b
 * @param a time to subtract from
 * @param b time to subtract
 * @return difference between a and b in microseconds
 */
inline long int
time_diff_usec(const timeval &a, const timeval &b)
{
  return (a.tv_sec - b.tv_sec) * 1000000 + (a.tv_usec - b.tv_usec);
}

class Clock;

class Time
{
 friend class Clock;
 public:
  Time();
  Time(const timeval* tv);
  Time(long ms);
  Time(float sec);
  Time(Clock *clock);
  Time(const Time &t);
  ~Time();

  float in_sec() const;
  long  in_msec() const;
  long  in_usec() const;
  const timeval* get_timeval() const;

  void set_time(const timeval* tv);
  void set_time(long ms);
  void set_time(float sec);

  Time & stamp();

  Time   operator+(const float sec) const;
  Time   operator+(const Time& t) const;
  Time   operator-(const Time& t) const;
  float  operator-(const Time* t) const;
  Time & operator+=(const long int usec);
  Time & operator+=(const Time& t);
  Time & operator-=(const Time& t);
  Time & operator=(const Time& t);

  const char * str();
  void         str_r(char *s);

 private:
  Clock   *clock;
  timeval  time;
  char     timestr[26]; // 26 as described in asctime_r() docs     
};

#endif
