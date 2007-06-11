
/***************************************************************************
 *  watch.cpp - A stopwatch
 *
 *  Generated: Sun June 03 15:38:24 2007
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

#include <utils/system/watch.h>
#include <utils/system/clock.h>
#include <utils/system/time.h>

/** @class Watch utils/system/watch.h
 * This is a stop-watch. Also, one can request the current time from the 
 * clock. Every watch counts time w.r.t. a certain time source.
 * @author Daniel Beck
 */


/** Constructor.
 * @param sel Select the time source from which you want to obtain the time.
 * This can either be the default-, the external-, or the system-time. The
 * selection cannot be changed later.
 */
Watch::Watch(Clock::TimesourceSelector sel)
{
  clock = Clock::init();

  is_running = false;
  is_paused = false;

  ts_sel = sel;
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
  clock->get_time(&now, ts_sel);

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
    t->set_time(&now);;
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
  clock->get_time(&now, ts_sel);
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
  clock->get_time(&now, ts_sel);

  if (!is_paused) {
    pause_start.set_time(&now);
    is_paused = true;
  }

  if (0 != t) {
    t->set_time(&now);;
  }
}


/** Returns the current watch time.
 * @return the current watch time
 */
Time
Watch::watch_time()
{
  timeval now;
  clock->get_time(&now, ts_sel);

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
  clock->get_time(&now, ts_sel);
  Time t(&now);
  return t;
}
