
/***************************************************************************
 *  time.c - A time class
 *
 *  Created: Wed Jun 06 16:50:11 2007
 *  Copyright  2007       Daniel Beck
 *             2007-2009  Tim Niemueller [www.niemueller.de]
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

#include <utils/time/time.h>
#include <utils/time/clock.h>

#include <core/exception.h>
#include <core/exceptions/software.h>

#include <time.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Time <utils/time/time.h>
 * A class for handling time.
 * @author Daniel Beck
 * @author Tim Niemueller
 *
 * @fn const timeval * Time::get_timeval() const
 * Obtain the timeval where the time is stored.
 * @return a const pointer to the timeval where the time is stored
 *
 * @fn long Time::get_sec() const
 * Get seconds.
 * @return seconds stored in time stamp
 *
 * @fn long Time::get_msec() const
 * Get milliseconds.
 * @return milliseconds stored in time stamp
 *
 * @fn long Time::get_usec() const
 * Get microseconds.
 * @return microseconds stored in time stamp
 *
 * @fn void Time::get_timestamp(long &sec, long &usec) const
 * Get time stamp.
 * @param sec upon return contains seconds stored in time stamp
 * @param usec upon return contains microseconds stored in time stamp
 */

/** Maximum size of string returned by str() and the minimum size
 * of the string passwd to str_r(). */
// as recommened in asctime_r() docs
const unsigned int Time::TIMESTR_SIZE = 26;


/** Constructor.
 * Sets time to the current time.
 */
Time::Time()
{
  __clock = Clock::instance();
  __clock->get_time(&__time);
  __timestr = NULL;
}


/** Constructor.
 * Sets time to the given time.
 * @param tv the Time object is initialized with the time given in this timeval
 */
Time::Time(const timeval* tv)
{
  __time.tv_sec = tv->tv_sec;
  __time.tv_usec = tv->tv_usec;
  __clock = Clock::instance();
  __timestr = NULL;
}


/** Constructor.
 * Sets time to the given time. Basically the same as setting from a timeval struct
 * but the components are given separately.
 * @param sec time in seconds since the epoch (or time range)
 * @param usec fractions in microseconds added to sec
 * @param clock optional clock to use, if NULL Clock::instance() will be used
 */
Time::Time(long sec, long usec, Clock *clock)
{
  __time.tv_sec  = sec;
  __time.tv_usec = usec;
  if (clock) {
    __clock = clock;
  } else {
    __clock = Clock::instance();
  }
  __timestr = NULL;
}


/** Constructor.
 * Sets time to given number of ms, use for time range.
 * @param ms the Time object is initialized to the time given in milli-seconds
 */
Time::Time(long ms)
{
  time_t sec = (time_t) (ms / 1000.0);
  suseconds_t usec =  (ms % 1000) * 1000;

  __time.tv_sec = sec;
  __time.tv_usec = usec;
  __clock = Clock::instance();
  __timestr = NULL;
}


/** Constructor.
 * Sets time to given number of ms, use for time range.
 * @param s the Time object is initialized to the time given in seconds
 */
Time::Time(float s)
{
  time_t sec = (time_t) s;
  suseconds_t usec = (suseconds_t)roundf((s - sec) * 1000000.f);

  __time.tv_sec = sec;
  __time.tv_usec = usec;
  __clock = Clock::instance();
  __timestr = NULL;
}


/** Constructor.
 * This constructor uses the supplied clock for setting the time. The
 * time is set to the current time.
 * @param clock clock
 */
Time::Time(Clock *clock)
{
  this->__clock = clock;
  __clock->get_time(&__time);
  __timestr = NULL;
}


/** Copy constructor.
 * @param t time to copy
 */
Time::Time(const Time &t)
{
  __time.tv_sec  = t.__time.tv_sec;
  __time.tv_usec = t.__time.tv_usec;
  __clock        = t.__clock;
  if (t.__timestr) {
    __timestr = (char *)malloc(TIMESTR_SIZE);
    strncpy(__timestr, t.__timestr, TIMESTR_SIZE);
  } else {
    __timestr = NULL;
  }
}


/** Copy constructor.
 * @param t time to copy
 */
Time::Time(const Time *t)
{
  __time.tv_sec  = t->__time.tv_sec;
  __time.tv_usec = t->__time.tv_usec;
  __clock        = t->__clock;
  if (t->__timestr) {
    __timestr = (char *)malloc(TIMESTR_SIZE);
    strncpy(__timestr, t->__timestr, TIMESTR_SIZE);
  } else {
    __timestr = NULL;
  }
}


/** Destructor. */
Time::~Time()
{
  if (__timestr)  free(__timestr);
}


/** Convet time to seconds.
 * Convert the stored time in a floating point number representing the
 * number of seconds. For a time the integral part is the number of seconds
 * since the epoch, for ranges you get the value as a float second.
 * @return the time in seconds
 */
float
Time::in_sec() const
{
  return (__time.tv_sec + __time.tv_usec / 1000000.f);
}


/** Convert the stored time into milli-seconds.
 * @return the time in milli-seconds
 */
long
Time::in_msec() const
{
  return (__time.tv_sec * 1000 + (long) (__time.tv_usec / 1000));
}


/** Convert the stored time into micro-seconds.
 * @return the time in micro-seconds
 */
long
Time::in_usec() const
{
  return (__time.tv_sec * 1000000 + __time.tv_usec);
}


/** Sets the time.
 * @param tv set the time to this value
 */
void
Time::set_time(const timeval* tv)
{
  __time.tv_sec = tv->tv_sec;
  __time.tv_usec = tv->tv_usec;
}


/** Sets the time.
 * @param sec seconds part of the time
 * @param usec microseconds part of the time
 */
void
Time::set_time(long int sec, long int usec)
{
  __time.tv_sec  = sec;
  __time.tv_usec = usec;
}


/** Sets the time.
 * @param ms set the time to this value
 */
void
Time::set_time(long ms)
{
  __time.tv_sec  = (time_t) (ms / 1000.0);
  __time.tv_usec = (ms % 1000) * 1000;
}


/** Sets the time.
 * @param s set the time to this value
 */
void
Time::set_time(float s)
{
  __time.tv_sec  = (time_t)floor(s);
  __time.tv_usec = (suseconds_t)(s - __time.tv_sec) * 1000000;
}

/** Set time to given time.
 * this is equivalent to operator+, but can be used in situations where
 * the operator cannot be used (for example in Lua).
 * @param t time to set to
 */
void
Time::set_time(const Time &t)
{
  *this = t;
}


/** Set time to given time.
 * @param t time to set to
 */
void
Time::set_time(const Time *t)
{
  __time.tv_sec  = t->__time.tv_sec;
  __time.tv_usec = t->__time.tv_usec;
}


/** Set clock for this instance.
 * @param clock clock to use from now on
 */
void
Time::set_clock(Clock *clock)
{
  if (clock == NULL) throw NullPointerException("Clock may not be NULL");
  __clock = clock;
}


/** Add seconds.
 * The effect is equivalent to operator+=(const float sec), but this
 * can be used when the operator is not available (i.e. wrapper languages)
 * and it does not return itself.
 * @param seconds time in seconds to add
 */
void
Time::add(float seconds)
{
  *this += seconds;
}

/** Operator that adds times.
 * @param t the other summand
 * @return the sum
 */
Time
Time::operator+(const Time& t) const
{
  Time ret;
  if (__time.tv_usec + t.__time.tv_usec >= 1000000)
  {
    ret.__time.tv_usec = __time.tv_usec + t.__time.tv_usec - 1000000;
    ret.__time.tv_sec = __time.tv_sec + t.__time.tv_sec + 1;
  }
  else
  {
    ret.__time.tv_usec = __time.tv_usec + t.__time.tv_usec;
    ret.__time.tv_sec = __time.tv_sec + t.__time.tv_sec;
  }

  return ret;
}


/** Operator that adds times.
 * @param t the other summand
 * @return the sum
 */
Time
Time::operator+(const Time* t) const
{
  return *this + *t;
}


/** Operator that adds times.
 * @param sec number of seconds to add
 * @return the sum
 */
Time
Time::operator+(const float sec) const
{
  Time ret;
  time_t sec_only = (time_t)floor(sec);
  suseconds_t usec_only = (suseconds_t)roundf((sec - sec_only) * 1000000);
  if ((__time.tv_usec + usec_only) >= 1000000)
  {
    ret.__time.tv_usec = __time.tv_usec + usec_only - 1000000;
    ret.__time.tv_sec = __time.tv_sec + sec_only + 1;
  }
  else
  {
    ret.__time.tv_usec = __time.tv_usec + usec_only;
    ret.__time.tv_sec = __time.tv_sec + sec_only;
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
  if (__time.tv_usec < t.__time.tv_usec)
  {
    ret.__time.tv_usec = 1000000 + __time.tv_usec - t.__time.tv_usec;
    ret.__time.tv_sec = __time.tv_sec - t.__time.tv_sec - 1;
  }
  else
  {
    ret.__time.tv_usec = __time.tv_usec - t.__time.tv_usec;
    ret.__time.tv_sec = __time.tv_sec - t.__time.tv_sec;
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
  return time_diff_sec(__time, t->__time);
}


/** += operator
 * @param t the other time
 * @return reference to this instance
 */
Time &
Time::operator+=(const Time& t)
{
  if (__time.tv_usec + t.__time.tv_usec >= 1000000)
  {
    __time.tv_usec += t.__time.tv_usec - 1000000;
    __time.tv_sec  += t.__time.tv_sec + 1;
  }
  else
  {
    __time.tv_usec += t.__time.tv_usec;
    __time.tv_sec  += t.__time.tv_sec;
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
  if ( __time.tv_usec + usec >= 1000000 )
  {
    //usec + __time.tv_usec might be more than 1 second
    long int tmp_usec = __time.tv_usec + usec;
    __time.tv_usec = tmp_usec % 1000000;
    __time.tv_sec  += tmp_usec / 1000000;
  }
  else
  {
    __time.tv_usec += usec;
  }

  return *this;
}


/** += operator for float seconds
 * @param sec number of seconds to add
 * @return the sum
 */
Time &
Time::operator+=(const float sec)
{
  time_t sec_only = (time_t)floor(sec);
  suseconds_t usec_only = (suseconds_t)roundf((sec - sec_only) * 1000000);
  if ((__time.tv_usec + usec_only) >= 1000000)
  {
    __time.tv_usec += usec_only - 1000000;
    __time.tv_sec  += sec_only + 1;
  }
  else
  {
    __time.tv_usec += usec_only;
    __time.tv_sec  += sec_only;
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
  __time.tv_sec  = t.__time.tv_sec;
  __time.tv_usec = t.__time.tv_usec;
  __clock        = t.__clock;
  return *this;
}


/** Check equality of times.
 * @param t time to compare to
 * @return true if sec and usec of both times are the same, false otherwise
 */
bool
Time::operator==(const Time& t) const
{
  return (__time.tv_sec == t.__time.tv_sec) &&
         (__time.tv_usec == t.__time.tv_usec);
}


/** Check equality of times.
 * @param t time to compare to
 * @return true if sec and usec of both times are the same, false otherwise
 */
bool
Time::operator==(const Time* t) const
{
  return (__time.tv_sec == t->__time.tv_sec) &&
         (__time.tv_usec == t->__time.tv_usec);
}


/** Check inequality of times.
 * @param t time to compare to
 * @return true if sec or usec of both times are different, false otherwise
 */
bool
Time::operator!=(const Time& t) const
{
  return (__time.tv_sec != t.__time.tv_sec) ||
         (__time.tv_usec != t.__time.tv_usec);
}


/** Check inequality of times.
 * @param t time to compare to
 * @return true if sec or usec of both times are different, false otherwise
 */
bool
Time::operator!=(const Time* t) const
{
  return (__time.tv_sec != t->__time.tv_sec) ||
         (__time.tv_usec != t->__time.tv_usec);
}


/** Set this time to the current time.
 * @return reference to this instance
 */
Time &
Time::stamp()
{
  if ( NULL != __clock ) {
    __clock->get_time(&__time);
  } else {
    throw Exception("Clock not set, cannot stamp time");
  }
  return *this;
}


/** Set this time to the current system time.
 * This bypasses any possibly registered time source. Use with care and only
 * where you really need the system time.
 * @return reference to this instance
 */
Time &
Time::stamp_systime()
{
  if ( NULL != __clock ) {
    __clock->get_systime(&__time);
  } else {
    throw Exception("Clock not set, cannot stamp time (systime)");
  }
  return *this;
}


/** Wait (sleep) for this time.
 * This waits for as much time as this instance provides. Note that you have to
 * make sure that you call this on a sensible time range. You probably do not want
 * to wait for almost 40 years when passing a time point...
 */
void
Time::wait()
{
  Time until, now;
  until += *this;

  // we want to release run status at least shortly
  usleep(0);

  long int remaining_usec = (until - now).in_usec();
  while ( remaining_usec > 0 ) {
    usleep(remaining_usec);
    now.stamp();
    remaining_usec = (until - now).in_usec();
  }
}


/** Wait (sleep) for this system time.
 * This waits for as much time as this instance provides. Unlike wait() this
 * method calculates the time in system time, althouh the main clock may run
 * slower for example in a simulation. Note that you have to make sure that you
 * call this on a sensible time range. You probably do not want to wait for
 * almost 40 years when passing a time point...
 */
void
Time::wait_systime()
{
  Time until, now;

  __clock->get_systime(until);
  until += *this;

  __clock->get_systime(now);

  // we want to release run status at least shortly
  usleep(0);

  long int remaining_usec = (until - now).in_usec();
  while ( remaining_usec > 0 ) {
    usleep(remaining_usec);
    __clock->get_systime(now);
    remaining_usec = (until - now).in_usec();
  }
}

/** Output function.
 * @return a pointer to a member containing a string represenation of
 * the given time. If seconds is smaller than 1 billion it is assumed that
 * this time represents a time range rather than a point in time and
 * the time is formatted as seconds.microseconds, otherwise the time
 * is formatted either via localtime() (if utc is false) or gmtime (if utc
 * is true).
 * @param utc true to get type formatted in UTC, otherwise local time
 */
const char *
Time::str(bool utc) const
{
  // allocate time string if not done yet
  if ( ! __timestr )  __timestr = (char *)malloc(TIMESTR_SIZE);

  // heuristic to distinguish times and time ranges
  if (__time.tv_sec < 1000000000) {
#ifdef __FreeBSD__
    snprintf(__timestr, TIMESTR_SIZE, "%i:%li", __time.tv_sec, __time.tv_usec);
#else
    snprintf(__timestr, TIMESTR_SIZE, "%li:%li", __time.tv_sec, __time.tv_usec);
#endif
  } else {
    tm time_tm;
    if ( utc ) {
      gmtime_r( &(__time.tv_sec), &time_tm );
    } else {
      localtime_r( &(__time.tv_sec), &time_tm );
    }
    asctime_r(&time_tm, __timestr);
    __timestr[strlen(__timestr) - 1] = 0;
  }

  return __timestr;
}


/** Output function.
 * This is the thread-safe version of str().
 * @param s pointer to a string of at least TIMESTR_SIZE bytes.
 * @param utc true to get type formatted in UTC, otherwise local time
 */
void
Time::str_r(char *s, bool utc)
{
  // heuristic to distinguish times and time ranges
  if (__time.tv_sec < 1000000000) {
#ifdef __FreeBSD__
    snprintf(s, TIMESTR_SIZE, "%i:%li", __time.tv_sec, __time.tv_usec);
#else
    snprintf(s, TIMESTR_SIZE, "%li:%li", __time.tv_sec, __time.tv_usec);
#endif
  } else {
    tm time_tm;
    if ( utc ) {
      gmtime_r( &(__time.tv_sec), &time_tm );
    } else {
      localtime_r( &(__time.tv_sec), &time_tm );
    }
    asctime_r(&time_tm, s);
    s[strlen(s) - 1] = 0;
  }
}

} // end namespace fawkes
