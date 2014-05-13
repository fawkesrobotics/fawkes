
/***************************************************************************
 *  simts.cpp - Simulator time source
 *
 *  Created: Mon Feb 25 15:49:16 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include <utils/time/simts.h>
#include <cstddef>

namespace fawkes {

/** @class SimulatorTimeSource <utils/time/simts.h>
 * Simulation time source.
 * This class is an utility to provide a generic time source for time in a simulated
 * environment. It can be restarted at an arbitrary time with an arbitrary offset.
 * It will then read the current real system time and save the initial offset. Each
 * time you query the time source it will return a given fixed time. The time is advanced
 * by setting a new offset (usually in every cycle).
 *
 * This implementation is rather primitive at the moment and could use some love.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
SimulatorTimeSource::SimulatorTimeSource()
{
  clock = Clock::instance();
  clock->get_systime(start_time);
  start_simoffset = 0;
  current_simtime = start_time;
}

/** Destructor. */
SimulatorTimeSource::~SimulatorTimeSource()
{
}


void
SimulatorTimeSource::get_time(timeval *tv) const
{
  if ( tv != NULL ) {
    const timeval *curt = current_simtime.get_timeval();
    tv->tv_sec  = curt->tv_sec;
    tv->tv_usec = curt->tv_usec;
  }
}


timeval
SimulatorTimeSource::conv_to_realtime(const timeval *tv) const
{
  float simdiff  = current_simoffset - start_simoffset;
  float realdiff = current_realtime - &start_time;

  float sim_to_real = realdiff / simdiff;

  Time query_simtime(tv);
  query_simtime -= start_time;
  float query_simtime_offset = query_simtime.in_sec() - start_simoffset;

  query_simtime_offset *= sim_to_real;

  Time final(query_simtime_offset);
  final += start_time;

  return *(final.get_timeval());;
}


timeval
SimulatorTimeSource::conv_native_to_exttime(const timeval *tv) const
{
  timeval rv = *tv;
  return rv;
}

/** Set start time.
 * @param initial_offset initial offset in seconds
 */
void
SimulatorTimeSource::set_start(float initial_offset)
{
  clock->get_systime(start_time);
  start_simoffset = initial_offset;
  current_simtime = start_time;
  //printf("Start time: %s  Start offset: %f\n", start_time.str(), start_simoffset);
}


/** Set simulation offset.
 * @param sim_offset simulation offset in seconds.
 */
void
SimulatorTimeSource::set_sim_offset(float sim_offset)
{
  clock->get_systime(current_realtime);
  current_simtime = start_time + (sim_offset - start_simoffset);
  current_simoffset = sim_offset;
  //printf("New current real time: %s  New current simtime: %s   new offset: %f\n",
  //       start_time.str(), current_simtime.str(), current_simoffset);
}

} // end namespace fawkes
