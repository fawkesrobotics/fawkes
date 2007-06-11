
/***************************************************************************
 *  clock.cpp - A central clock
 *
 *  Generated: Sun June 03 00:23:59 2007
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

#include <utils/system/clock.h>
#include <utils/system/timesource.h>
#include <utils/logging/liblogger.h>

/** @class Clock utils/system/clock.h
 * This is supposed to be the central (wall-) clock in Fawkes.
 * It is implemented as a singleton to ensure that there is only
 * one object. So-called TimeSources can be registered at the Clock
 * their current time can be retrieved through the Clock.
 * @author Daniel Beck
 */

/** initialize static members */
Clock* Clock::instance = 0;
bool Clock::destruct_ok = false;

/** Constructor. */
Clock::Clock()
{
  destruct_ok = false;
  ext_timesource = 0;
  ext_default = false;
}


/** Destructor.
 * Generates a log message if Clock::finalize wasn't called beforehand 
 */
Clock::~Clock()
{
  if ( !destruct_ok ) {
    LibLogger::log_error("Clock", "dtor called without calling Clock::finalize() first");
  }

  free(ext_timesource);
}


/** Clock initializer.
 * This one is static and has to be called to instantiate a Clock object.
 * In further calls it just returns a pointer to the Clock object.
 * @return a pointer to the Clock object
 */
Clock*
Clock::init()
{
  if ( 0 == instance )
    {
      instance = new Clock();
    }
  
  return instance;
}


/** Finalize.
 * Sets a boolean flag which prevents a log message from being generated in
 * the destructor.
 */
void
Clock::finalize()
{
  destruct_ok = true;
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
  free(ext_timesource);

  ext_timesource = ts;

  if (make_default) {
    ext_default = true;
  }
}


/** Makes the external time source the default time source.
 */
void
Clock::make_ext_default_timesource()
{
  if (0 != ext_timesource) {
    ext_default = true;
  } else {
    LibLogger::log_warn("Clock", "Trying to make the external timesource the default timesource but there is no external timesource");
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
 * @return returns false if no external time source is registered
 */
bool
Clock::get_time(struct timeval* tv, TimesourceSelector sel) const
{
  if ( (DEFAULT == sel && !ext_default) ||
       REALTIME == sel)
    {
      gettimeofday(tv, 0);
      return true;
    }
  else if ( (EXTERNAL == sel) && 
	    (0 == ext_timesource) )
    {
      gettimeofday(tv, 0);
      return false;
    }
  else
    {
      ext_timesource->get_time(tv);
      return true;
    }
}


/** Convert a time given w.r.t. the external time source into the system time.
 * @param t the time that is converted to the system time
 * @return the time in system time
 */ 
Time
Clock::ext_to_realtime(const Time& t)
{
  timeval tv;
  Time ret;
  
  if (0 != ext_timesource) 
    {
      tv = ext_timesource->conv_to_realtime(t.get_timeval());
      ret.set_time(&tv); 
    }
  else 
    {
      ret = t;
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
