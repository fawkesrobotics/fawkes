
/***************************************************************************
 *  sensor_thread.cpp - Katana plugin sensor thread
 *
 *  Created: Fri Jun 12 15:00:33 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
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

using namespace fawkes;

/** @class KatanaSensorThread "sensor_thread.h"
 * Katana sensor thread.
 * This thread integrates into the Fawkes main loop at the SENSOR hook and
 * triggers the act thread to retrieve and write new sensor data.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param act_thread Katana act thread instance to trigger for the sensor
 * update.
 */
KatanaSensorThread::KatanaSensorThread(KatanaActThread *act_thread)
  : Thread("KatanaSensorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
  __act_thread = act_thread;
}

void
KatanaSensorThread::loop()
{
  __act_thread->update_sensor_values();
}
