
/***************************************************************************
 *  watch.cpp - A stopwatch
 *
 *  Generated: Sun June 03 15:38:24 2007
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

#include <utils/time/watch.h>
#include <utils/time/clock.h>
#include <utils/time/time.h>

namespace fawkes {

/** @class Watch <utils/time/watch.h>
 * This is a stop-watch. Also, one can request the current time from the 
 * clock. Every watch counts time w.r.t. a certain time source.
 * @author Daniel Beck
 */


/** Constructor.
 * @param clock clock instance to use for measurement.
 */
Watch::Watch(Clock *clock)
{
  this->clock = clock;

  is_running = false;
  is_paused = false;

}


/** Destructor. */
Watch::~Watch()
{
}


/** Starts the watch.
 * This starts the watch. In case it is paused, currently, the watch
 * is restarted
 * @param t the time at which the watch is started is written to this time object
 */
void
Watch::start(Time* t)
{
  timeval now;
  clock->get_time(&now);

  if (is_running && is_paused)
    {
      pause_stop.set_time(&now);
      is_paused = false;
  
      pause_time += pause_stop - pause_start;
    }
  else if (!is_running)
    {
      pause_time.set_time( 0.0f );
      is_running = true;

      watch_start.set_time(&now);
    }
  else /* is_running && !is_paused */
    {
      // todo
    }

  if (0 != t) {
    t->set_time(&now);
  }
}


/** Stops the watch.
 * This stops the watch also when it is paused, currently
 * @param t the time at which the watch is started is written to this time object
 */
void
Watch::stop(Time* t)
{
  timeval now;
  clock->get_time(&now);
  watch_stop.set_time(&now);
  is_running = false;

  if (is_paused)
    {
      pause_stop.set_time(&now);
      pause_time += pause_stop - pause_start;

      is_paused = false;
    }

  if (0 != t) {
    t->set_time(&now);
  }
}


/** Pauses the watch.
 * Puts the watch into pause mode
 * @param t the time at which the watch is started is written to this time object
 */
void
Watch::pause(Time* t)
{
  timeval now;
  clock->get_time(&now);

  if (!is_paused) {
    pause_start.set_time(&now);
    is_paused = true;
  }

  if (0 != t) {
    t->set_time(&now);;
  }
}


/** Reset time. */
void
Watch::reset()
{
  timeval now;
  clock->get_time(&now);  
  watch_start.set_time(&now);
}


/** Returns the current watch time.
 * @return the current watch time
 */
Time
Watch::watch_time()
{
  timeval now;
  clock->get_time(&now);

  Time ret(&now);

  if (is_running && !is_paused) 
    {
      ret -= watch_start + pause_time;
    }
  else if (is_running && is_paused) 
    {
      Time cur_pause;
      cur_pause = ret - pause_start;
      ret -= watch_start + pause_time + cur_pause;
    }
  else
    {
      ret = watch_stop - watch_start - pause_time;
    }

  return ret;
}


/** Returns the current clock time.
 * @return the current clock time 
 */
Time
Watch::clock_time()
{
  timeval now;
  clock->get_time(&now);
  Time t(&now);
  return t;
}

} // end namespace fawkes
