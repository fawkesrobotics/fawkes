
/***************************************************************************
 *  sensor_thread.cpp - Robotino sensor thread
 *
 *  Created: Sun Nov 13 15:35:24 2011
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
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
#include "com_thread.h"

using namespace fawkes;

/** @class RobotinoSensorThread "sensor_thread.h"
 * Robotino sensor hook integration thread.
 * This thread integrates into the Fawkes main loop at the SENSOR hook and
 * writes new sensor data.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param com_thread communication thread to trigger for writing data
 */
RobotinoSensorThread::RobotinoSensorThread(RobotinoComThread *com_thread)
	: Thread("RobotinoSensorThread", Thread::OPMODE_WAITFORWAKEUP),
	  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
	com_ = com_thread;
}


void
RobotinoSensorThread::init()
{
}


void
RobotinoSensorThread::finalize()
{
}

void
RobotinoSensorThread::loop()
{
	com_->update_bb_sensor();
}
