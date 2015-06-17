
/***************************************************************************
 *  sensor_thread.cpp - Robotis dynamixel servo sensor integration thread
 *
 *  Created: Mon Mar 23 20:20:02 2015 (based on pantilt plugin)
 *  Copyright  2006-2015  Tim Niemueller [www.niemueller.de]
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

#include "sensor_thread.h"
#include "driver_thread.h"

using namespace fawkes;

/** @class DynamixelSensorThread "sensor_thread.h"
 * Robotis dynamixel servo sensor thread.
 * This thread integrates into the Fawkes main loop at the SENSOR hook and
 * triggers the driver thread to retrieve and write new sensor data.
 * @author Tim Niemueller
 */

/** Constructor. */
DynamixelSensorThread::DynamixelSensorThread()
  : Thread("DynamixelSensorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
}


void
DynamixelSensorThread::loop()
{
  for (auto &d : driver_threads_) {
    d->exec_sensor();
  }
}

/** Add a driver thread to wake in SENSOR_ACQUIRE hook.
 * @param drv_thread driver thread to add
 */
void
DynamixelSensorThread::add_driver_thread(DynamixelDriverThread *drv_thread)
{
  driver_threads_.push_back(drv_thread);
}
