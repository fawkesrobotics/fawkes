
/***************************************************************************
 *  act_thread.cpp - Pan/tilt plugin act thread (abstract)
 *
 *  Created: Thu Jun 18 09:48:03 2009
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

#include "act_thread.h"

using namespace fawkes;

/** @class PanTiltActThread "act_thread.h"
 * Pan/tilt act thread.
 * This thread integrates into the Fawkes main loop at the ACT_EXEC hook and
 * interacts with the controller of a particular pan/tilt unit. Note that this
 * abstract class is then implemented by threads interacting with a particular
 * pan/tilt unit.
 *
 * Each implementation of this thread shall open an instance of a PanTiltInterface
 * and act within this thread's loop() method. The data read back from the unit
 * shall be written in the update_sensor_values() method, that is called from
 * the PanTiltSensorThread.
 *
 * @author Tim Niemueller
 *
 * @fn void PanTiltActThread::update_sensor_values()
 * Update sensor values of the PTU in the BB interface.
 */

/** Constructor.
 * @param thread_name thread name
 */
PanTiltActThread::PanTiltActThread(const char *thread_name)
  : Thread(thread_name, Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC)
{
}


/** Destructor. */
PanTiltActThread::~PanTiltActThread()
{
}
