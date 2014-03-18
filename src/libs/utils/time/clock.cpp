
/***************************************************************************
 *  clock.cpp - A central clock
 *
 *  Created: Sun Jun 03 00:23:59 2007
 *  Copyright  2007       Daniel Beck 
 *             2007-2008  Tim Niemueller [www.niemueller.de]
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

#include <utils/time/clock.h>
#include <utils/time/timesource.h>
#include <core/exception.h>

#include <cstdlib>

namespace fawkes {

/** @class Clock clock.h <utils/time/clock.h>
 * This is supposed to be the central clock in Fawkes.
 * It is implemented as a singleton to ensure that there is only
 * one object. So-called TimeSources can be registered at the Clock
 * their current time can be retrieved through the Clock.
 * @author Daniel Beck, Tim Niemueller
 */

/** initialize static members */
Clock* Clock::_instance = NULL;

/** Constructor. */
Clock::Clock()
{
  ext_timesource = 0;
  ext_default = false;
}


/** Destructor. */
Clock::~Clock()
{
  delete ext_timesource;
}


/** Clock initializer.
 * This one is static and has to be called to instantiate a Clock object.
 * In further calls it just returns a pointer to the Clock object.
 * @return a pointer to the Clock object
 */
Clock *
Clock::instance()
{
  if (NULL == _instance) {
    _instance = new Clock();
  }
  
  return _instance;
}


/** Finalize. */
void
Clock::finalize()
{
  delete _instance;
  _instance = NULL;
}


/** Register an external time source.
 * 
 * @param ts a pointer to the external time source
 * @param make_default if true, this time source is made the default
 * timesource which means that for every call of get_time() the time
 * of the external time source is returned
 */
void
Clock::register_ext_timesource(TimeSource* ts, bool make_default)
{
  ext_timesource = ts;

  if (make_default) {
    ext_default = true;
  }
}


/** Remove external time source.
 * If an external timesource is currently set it is removed. The time source
 * will not be deleted but only the reference to it is removed.
 * @param ts only remove time source if it equals ts, if NULL remove no matter what.
 */
void
Clock::remove_ext_timesource(TimeSource *ts)
{
  if ( (ts == NULL) || (ext_timesource == ts) ) {
    ext_timesource = NULL;
    ext_default = false;
  } else {
    throw Exception("Time sources do not match. Not removing.");
  }
}


/** Set/unset the external time source as the default time source.
 * @param ext_is_default true to make external time source the default,
 * false to disable it as the default.
 */
void
Clock::set_ext_default_timesource(bool ext_is_default)
{
  if ( ext_is_default ) {
    if (NULL != ext_timesource) {
      ext_default = true;
    } else {
      throw Exception("Trying to make the external timesource the default timesource but there is no external timesource");
    }
  } else {
    ext_default = false;
  }
}


/** Checks whether the external time source is the default time soucre.
 * @return true if external time source is default time source
 */
bool
Clock::is_ext_default_timesource() const
{
  return ext_default;
}


/** Returns the time of the selected time source.
 * @param tv pointer to a timeval struct where the time is written to
 * @param sel allows to select the time source
 */
void
Clock::get_time(struct timeval* tv, TimesourceSelector sel) const
{
  if ( (DEFAULT == sel && !ext_default) ||
       REALTIME == sel)
    {
      gettimeofday(tv, 0);
    }
  else if ( (EXTERNAL == sel) && 
	    (NULL == ext_timesource) )
    {
      throw Exception("No external time source registered");
    }
  else
    {
      ext_timesource->get_time(tv);
    }
}


/** Returns the time of the selected time source.
 * @param tv pointer to a timeval struct where the time is written to
 */
void
Clock::get_time(struct timeval* tv) const
{
  if ( ext_default ) {
    if ( NULL == ext_timesource ) {
      throw Exception("No external time source registered");
    }
    ext_timesource->get_time(tv);
  } else {
    gettimeofday(tv, NULL);
  }
}


/** Returns the time of the selected time source.
 * @param time reference to a time where the result is stored
 */
void
Clock::get_time(Time &time) const
{
  get_time(&(time.__time));
}




/** Returns the time of the selected time source.
 * @param time reference to a time where the result is stored
 * @param sel allows to select the time source
 */
void
Clock::get_time(Time &time, TimesourceSelector sel) const
{
  get_time(&(time.__time), sel);
}


/** Returns the time of the selected time source.
 * @param time pointer to a Time instance
 */
void
Clock::get_time(Time *time) const
{
  get_time(&(time->__time));
}




/** Returns the time of the selected time source.
 * @param time pointer to a Time instance where the time is stored
 * @param sel allows to select the time source
 */
void
Clock::get_time(Time *time, TimesourceSelector sel) const
{
  get_time(&(time->__time), sel);
}


/** Returns the system time.
 * @param tv pointer to a timeval struct where the time is written to
 */
void
Clock::get_systime(struct timeval* tv) const
{
  gettimeofday(tv, 0);
}


/** Returns the time of the selected time source.
 * @param time reference to Time instance where the time is stored
 */
void
Clock::get_systime(Time &time) const
{
  gettimeofday(&(time.__time), 0);
}


/** Returns the time of the selected time source.
 * @param time pointer to Time instance where the time is stored
 */
void
Clock::get_systime(Time *time) const
{
  gettimeofday(&(time->__time), 0);
}


/** Get the current time.
 * @return current time
 */
Time
Clock::now() const
{
  Time t(_instance);
  return t.stamp();
}


/** How much time has elapsed since t?
 * Calculated as "now - t" in seconds.
 * @param t time
 * @return elapsed seconds
 */
float
Clock::elapsed(Time *t) const
{
  Time nowt(_instance);
  return nowt - t;
}


/** How much system time has elapsed since t?
 * Use only for system time criteria like timeouts.
 * @param t time
 * @return elapsed system seconds
 */
float
Clock::sys_elapsed(Time *t) const
{
  struct timeval nowt;
  gettimeofday(&nowt, NULL);
  return time_diff_sec(nowt, t->__time);
}


/** Convert a time given w.r.t. the external time source into the system time.
 * @param t the time that is converted to the system time
 * @return the time in system time
 */ 
Time
Clock::ext_to_realtime(const Time& t)
{
  timeval tv;
  Time ret(t);
  
  if (NULL != ext_timesource) {
    tv = ext_timesource->conv_to_realtime(t.get_timeval());
    ret.set_time(&tv); 
  }

  return ret;
}


/** Convert some native time to a Fawkes time.
 * If communicating with some entity using some native time format (e.g.
 * a simulation giving offset timestamps from the simulation start) it
 * is necessary to convert them to a time that the time is comparable
 * to other times generated using the clock (and hence external timesource).
 * @param t the time that is to be converted to the a Fawkes time
 * @return the time in Fawkes time comparable to other times queried
 * from this lock. Identity of the given time if no external time source
 * has been registered.
 */ 
Time
Clock::native_to_time(const Time &t)
{
  timeval tv;
  Time ret(t);
  
  if (NULL != ext_timesource) {
      tv = ext_timesource->conv_native_to_exttime(t.get_timeval());
      ret.set_time(&tv); 
  }

  return ret;
}

/** Check whether an external time source is registered.
 * @return true if an external time source is registered
 */
bool
Clock::has_ext_timesource() const
{
  if (0 != ext_timesource) {
    return true;
  } else {
    return false;
  }
}

} // end namespace fawkes
