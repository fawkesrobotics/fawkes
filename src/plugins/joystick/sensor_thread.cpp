
/***************************************************************************
 *  sensor_thread.cpp - Joystick thread that pushes data into the interface
 *
 *  Created: Sat Nov 22 18:05:55 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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
#include "acquisition_thread.h"

#include <interfaces/JoystickInterface.h>

using namespace fawkes;

/** @class JoystickSensorThread "sensor_thread.h"
 * Joystick sensor thread.
 * This thread integrates into the Fawkes main loop at the sensor hook and
 * publishes new data when available from the JoystickAcquisitionThread.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param aqt JoystickAcquisitionThread to get data from
 */
JoystickSensorThread::JoystickSensorThread(JoystickAcquisitionThread *aqt)
  : Thread("JoystickSensorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
  __aqt    = aqt;
}


void
JoystickSensorThread::init()
{
  __joystick_if = blackboard->open_for_writing<JoystickInterface>("Joystick");
}


void
JoystickSensorThread::finalize()
{
  blackboard->close(__joystick_if);
}


void
JoystickSensorThread::loop()
{
  if ( __aqt->lock_if_new_data() ) {
    __joystick_if->set_num_axes( __aqt->num_axes() );
    __joystick_if->set_num_buttons( __aqt->num_buttons() );
    __joystick_if->set_pressed_buttons( __aqt->pressed_buttons() );
    __joystick_if->set_axis( __aqt->axis_values() );
    __joystick_if->write();
    __aqt->unlock();
  }
}
