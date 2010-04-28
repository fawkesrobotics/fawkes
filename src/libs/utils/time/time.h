
/***************************************************************************
 *  time.h - Time utils
 *
 *  Created: Wed Jan 18 15:56:33 2006 (from FireVision)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_TIME_TIME_H_
#define __UTILS_TIME_TIME_H_

#include <sys/time.h>

namespace fawkes {

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
  //double required if we do not want to loose the usecs
  double res = a.tv_sec  - b.tv_sec + (a.tv_usec - b.tv_usec) / 1000000.0;
  return res;
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
  //double required if we do not want to loose the usecs
  double res = a_sec - b_sec + (a_usec - b_usec) / 1000000.0;
  return res;
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
  Time(long sec, long usec, Clock *clock = 0);
  Time(long ms);
  Time(float sec);
  Time(Clock *clock);
  Time(const Time &t);
  Time(const Time *t);
  ~Time();

  float in_sec() const;
  long  in_msec() const;
  long  in_usec() const;

  const timeval * get_timeval() const { return &__time; }
  long            get_sec() const  { return __time.tv_sec; }
  long            get_msec() const { return __time.tv_usec * 1000; }
  long            get_usec() const { return __time.tv_usec; }
  void            get_timestamp(long &sec, long &usec) const
                  { sec  = __time.tv_sec; usec = __time.tv_usec; }

  void set_time(const timeval* tv);
  void set_time(long int sec, long int usec);
  void set_time(long ms);
  void set_time(float sec);
  void set_time(const Time &t);
  void set_time(const Time *t);

  void set_clock(Clock *clock);

  void add(float seconds);

  Time & stamp();
  Time & stamp_systime();

  Time   operator+(const float sec) const;
  Time   operator+(const Time& t) const;
  Time   operator+(const Time* t) const;
  Time   operator-(const Time& t) const;
  float  operator-(const Time* t) const;
  Time & operator+=(const long int usec);
  Time & operator+=(const Time& t);
  Time & operator+=(const float sec);
  Time & operator-=(const Time& t);
  Time & operator=(const Time& t);
  bool   operator==(const Time& t) const;
  bool   operator==(const Time* t) const;
  bool   operator!=(const Time& t) const;
  bool   operator!=(const Time* t) const;

  void wait();
  void wait_systime();

  const char * str(bool utc = false) const;
  void         str_r(char *s, bool utc = false);

  static const unsigned int TIMESTR_SIZE;

 private:
  Clock   *__clock;
  timeval  __time;
  mutable char    *__timestr;
};

} // end namespace fawkes

#endif
