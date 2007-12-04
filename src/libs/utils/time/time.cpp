
/***************************************************************************
 *  time.c - A time class
 *
 *  Created: Wed June 06 16:50:11 2007
 *  Copyright  2007  Daniel Beck
 *             2007  Tim Niemueller [www.niemueller.de]
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

#include <utils/time/time.h>
#include <utils/time/clock.h>

#include <core/exception.h>

#include <time.h>
#include <cmath>
#include <cstdio>

/** @class Time <utils/time/time.h>
 * A class for handling time.
 * @author Daniel Beck
 * @author Tim Niemueller
 */


/** Constructor.*/
Time::Time()
{
  time.tv_sec = 0;
  time.tv_usec = 0;
  clock = NULL;
}


/** Constructor. 
 * @param tv the Time object is initialized with the time given in this timeval
*/
Time::Time(const timeval* tv)
{
  time.tv_sec = tv->tv_sec;
  time.tv_usec = tv->tv_usec;
  clock = NULL;
}


/** Constructor. 
 * @param ms the Time object is initialized to the time given in milli-seconds
 */
Time::Time(long ms)
{
  time_t sec = (time_t) (ms / 1000.0);
  suseconds_t usec =  (ms % 1000) * 1000;

  time.tv_sec = sec;
  time.tv_usec = usec;
  clock = NULL;
}


/** Constructor. 
 * @param s the Time object is initialized to the time given in seconds
 */
Time::Time(float s)
{
  time_t sec = (time_t) s;
  suseconds_t usec = ((long)s - sec) * 1000000;

  time.tv_sec = sec;
  time.tv_usec = usec;
  clock = NULL;
}


/** Constructor.
 * This constructor uses the supplied clock for setting the time.
 * @param clock clock
 */
Time::Time(Clock *clock)
{
  this->clock = clock;
  clock->get_time(&time);
}


/** Copy constructor.
 * @param t time to copy
 */
Time::Time(const Time &t)
{
  time.tv_sec  = t.time.tv_sec;
  time.tv_usec = t.time.tv_usec;
  clock        = t.clock;
}


/** Destructor. */
Time::~Time()
{
}


/** Convert the stored time in a floating point number representing the number
 * of seconds.
 * @return the time in seconds
 */
float
Time::in_sec() const
{
  return (time.tv_sec + time.tv_usec / 1000000.f);
}


/** Convert the stored time into milli-seconds.
 * @return the time in milli-seconds
 */
long
Time::in_msec() const
{
  return (time.tv_sec * 1000 + (long) (time.tv_usec / 1000));
}


/** Convert the stored time into micro-seconds.
 * @return the time in micro-seconds
 */
long
Time::in_usec() const
{
  return (time.tv_sec * 1000000 + time.tv_usec);
}


/** Obtain the timeval where the time is stored.
 * @return a const pointer to the timeval where the time is stored
 */
const timeval *
Time::get_timeval() const
{
  return &time;
}


/** Sets the time.
 * @param tv set the time to this value
 */
void
Time::set_time(const timeval* tv)
{
  time.tv_sec = tv->tv_sec;
  time.tv_usec = tv->tv_usec;
}


/** Sets the time.
 * @param ms set the time to this value
 */
void
Time::set_time(long ms)
{
  time.tv_sec  = (time_t) (ms / 1000.0);
  time.tv_usec = (ms % 1000) * 1000;
}


/** Sets the time.
 * @param s set the time to this value
 */
void
Time::set_time(float s)
{
  time.tv_sec  = (time_t)floor(s);
  time.tv_usec = (suseconds_t)(s - time.tv_sec) * 1000000;
}


/** Operator that adds times.
 * @param t the other summand
 * @return the sum
 */
Time
Time::operator+(const Time& t) const
{
  Time ret;
  if (time.tv_usec + t.time.tv_usec > 1000000)
    {
      ret.time.tv_usec = time.tv_usec + t.time.tv_usec - 1000000;
      ret.time.tv_sec = time.tv_sec + t.time.tv_sec + 1;
    }
  else
    {
      ret.time.tv_usec = time.tv_usec + t.time.tv_usec;
      ret.time.tv_sec = time.tv_sec + t.time.tv_sec;
    }

  return ret;
}


/** Operator that substracts one Time from another.
 * @param t the Time that is substracted
 * @return the difference
 */
Time
Time::operator-(const Time& t) const
{
  Time ret;
  if (time.tv_usec < t.time.tv_usec)
    {
      ret.time.tv_usec = 1000000 + time.tv_usec - t.time.tv_usec;
      ret.time.tv_sec = time.tv_sec - t.time.tv_sec - 1;
    }
  else
    {
      ret.time.tv_usec = time.tv_usec - t.time.tv_usec;
      ret.time.tv_sec = time.tv_sec - t.time.tv_sec;
    }
  
  return ret;
}


/** Operator that substracts one Time from another.
 * @param t the Time that is substracted
 * @return the difference
 */
float
Time::operator-(const Time* t) const
{
  return time_diff_sec(time, t->time);
}


/** += operator 
 * @param t the other time
 * @return reference to this instance
*/
Time &
Time::operator+=(const Time& t)
{
  if (time.tv_usec + t.time.tv_usec > 1000000)
    {
      time.tv_usec += t.time.tv_usec - 1000000;
      time.tv_sec  += t.time.tv_sec + 1;
    }
  else
    {
      time.tv_usec += t.time.tv_usec;
      time.tv_sec  += t.time.tv_sec;
    }

  return *this;
}


/** += operator 
 * @param usec microseconds to add
 * @return reference to this instance
*/
Time &
Time::operator+=(const long int usec)
{
  if ( time.tv_usec + usec > 1000000 )
    {
      time.tv_usec += usec - 1000000;
      time.tv_sec  += 1;
    }
  else
    {
      time.tv_usec += usec;
    }

  return *this;
}


/** -= operator.
 * @param t the other time
 * @return reference to this instance
 */
Time &
Time::operator-=(const Time& t)
{
  *this = *this - t;
  return *this;
}


/** Assign operator.
 * @param t time to assign to this instance
 * @return reference to this instance
 */
Time &
Time::operator=(const Time &t)
{
  time.tv_sec  = t.time.tv_sec;
  time.tv_usec = t.time.tv_usec;
  clock        = t.clock;
  return *this;
}


/** Set this time to the current time.
 * @return reference to this instance
 */
Time &
Time::stamp()
{
  if ( NULL != clock ) {
    clock->get_time(&time);
  } else {
    throw Exception("Clock not set, cannot stamp time");
  }
  return *this;
}


/** Output function.
 * @return a pointer to a member containing a string represenation of
 * the given time. If seconds is smaller than 1 billion it is assumed that
 * this time represents a time range rather than a point in time.
 */
const char *
Time::str()
{
  tm time_tm;
  // heuristic to distinguish times and time ranges
  if (1000000000 < time.tv_sec) {
    localtime_r( &(time.tv_sec), &time_tm );
    snprintf(timestr, sizeof(timestr), "%li:%li", time.tv_sec, time.tv_usec);
  } else {
    gmtime_r( &(time.tv_sec), &time_tm );
    asctime_r(&time_tm, timestr);
  }

  return timestr;
}


/** Output function.
 * This is the thread-safe version of str().
 * @param s pointer to a string of at least 26 bytes.
 */
void
Time::str_r(char *s)
{
  tm time_tm;
  // heuristic to distinguish times and time ranges
  if (1000000000 < time.tv_sec) {
    localtime_r( &(time.tv_sec), &time_tm );
    snprintf(s, sizeof(timestr), "%li:%li", time.tv_sec, time.tv_usec);
  } else {
    gmtime_r( &(time.tv_sec), &time_tm );
    asctime_r(&time_tm, s);
  }
}
