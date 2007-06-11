
/***************************************************************************
 *  time.c - A time class
 *
 *  Created: Wed June 06 16:50:11 2007
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <utils/system/time.h>

#include <time.h>

#include <iomanip>

/** @class Time utils/system/time.h
 * A class for handling time.
 */


/** Constructor.*/
Time::Time()
{
  time.tv_sec = 0;
  time.tv_usec = 0;
}


/** Constructor. 
 * @param tv the Time object is initialized with the time given in this timeval
*/
Time::Time(const timeval* tv)
{
  time.tv_sec = tv->tv_sec;
  time.tv_usec = tv->tv_usec;
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
}


/** Destructor. */
Time::~Time()
{
}


/** Convert the stored time in a floating point number representing the number
 * of seconds.
 * @return the time in seconds
 */
float Time::in_sec() const
{
  float ret_val;
  ret_val = time.tv_sec + time.tv_usec / 1000000.0;
  
  return ret_val;
}


/** Convert the stored time into milli-seconds.
 * @return the time in milli-seconds
 */
long Time::in_msec() const
{
  long ret_val;
  ret_val = time.tv_sec * 1000 + (long) (time.tv_usec / 1000);
  
  return ret_val;
}


/** Obtain the timeval where the time is stored.
 * @return a const pointer to the timeval where the time is stored
 */
const timeval* Time::get_timeval() const
{
  return &time;
}


/** Sets the time.
 * @param tv set the time to this value
 */
void Time::set_time(const timeval* tv)
{
  time.tv_sec = tv->tv_sec;
  time.tv_usec = tv->tv_usec;
}


/** Sets the time.
 * @param ms set the time to this value
 */
void Time::set_time(long ms)
{
  time_t sec = (time_t) (ms / 1000.0);
  suseconds_t usec =  (ms % 1000) * 1000;

  time.tv_sec = sec;
  time.tv_usec = usec;
}


/** Sets the time.
 * @param s set the time to this value
 */
void Time::set_time(float s)
{
  time_t sec = (time_t) s;
  suseconds_t usec = ((long)s - sec) * 1000000;

  time.tv_sec = sec;
  time.tv_usec = usec;
}


/** Operator that adds times.
 * @param t the other summand
 * @return the sum
 */
Time Time::operator+(const Time& t) const
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
 * @return the differnce
 */
Time Time::operator-(const Time& t) const
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


/** += operator 
 * @param t the other time
*/
void Time::operator+=(const Time& t)
{
  *this = *this + t;
}


/** -= operator.
 * @param t the other time
 */
void Time::operator-=(const Time& t)
{
  *this = *this - t;
}


/** Output function.
 * @param ostr the stream to which output is written
 * @param t the Time
 * @return a reference to the stream
 */
std::ostream& operator<<(std::ostream& ostr, const Time& t)
{
  tm time_tm;
  if (1000000000 < t.time.tv_sec) {
    localtime_r( &(t.time.tv_sec), &time_tm );
  } else {
    gmtime_r( &(t.time.tv_sec), &time_tm );
  }

  bool not_null = false;

  if (0 != time_tm.tm_hour) 
    {
      ostr << std::setw(2) << std::setfill('0') << time_tm.tm_hour << "h";
      not_null = true;
    }

  if (0 != time_tm.tm_min || not_null) 
    {
      ostr << std::setw(2) << std::setfill('0') << time_tm.tm_min << "m";
      not_null = true;
    }

  if (0 != time_tm.tm_sec || not_null) 
    {
      ostr << std::setw(2) << std::setfill('0') << time_tm.tm_sec << "s";
      not_null = true;
    }

  ostr << std::setw(6) << std::setfill('0') << time_tm.tm_sec << "usec";

  return ostr;
}
