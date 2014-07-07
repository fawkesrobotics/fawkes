
/***************************************************************************
 *  sensor_thread.cpp - Robotino sensor thread
 *
 *  Created: Sun Nov 13 15:35:24 2011
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "sensor_com_handler.h"

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

using namespace fawkes;

/** @class RobotinoSensorComHandler "sensor_com_handler.h"
 * Handler for sensor information.
 * This class is used to receive events whenever new data has been
 * received via Robotino Com, in particular the sensor state.
 * This is required to avoid a starving lock when calling Com::sensorState().
 *
 * The sensor state is double buffered as to minimize delays and lock
 * times. The sensor state is also time-stamped as soon as it is received.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param clock clock for timestamps
 */
RobotinoSensorComHandler::RobotinoSensorComHandler(Clock *clock)
{
  active_state_ = 0;
  mutex_ = new Mutex();
  times_[0].set_clock(clock);
  times_[1].set_clock(clock);
}

/** Destructor. */
RobotinoSensorComHandler::~RobotinoSensorComHandler()
{
  delete mutex_;
}

/** Update event. */
void
RobotinoSensorComHandler::updateEvent()
{
  unsigned int next_state = 1 - active_state_;
  sensor_states_[next_state] = sensorState();
  times_[next_state].stamp();

  MutexLocker lock(mutex_);
  active_state_ = next_state;
}

/** Get current sensor state.
 * @param time upon return contains timestamp for sensor state
 * @return latest sensor state
 */
rec::iocontrol::remotestate::SensorState
RobotinoSensorComHandler::sensor_state(fawkes::Time &time) const
{
  MutexLocker lock(mutex_);
  time = times_[active_state_];
  return sensor_states_[active_state_];
}
