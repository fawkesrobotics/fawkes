
/***************************************************************************
 *  roombajoy_thread.cpp - Roomba joystick control thread
 *
 *  Created: Sat Jan 29 14:36:18 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "roombajoy_thread.h"
#include <interfaces/Roomba500Interface.h>
#include <interfaces/JoystickInterface.h>

#include <cstdlib>

using namespace fawkes;

/** @class RoombaJoystickThread "roombajoy_thread.h"
 * Roomba joystick control thread.
 * Read joystick information from the blackboard and transform it into
 * commands for the Roomba plugin.
 * This is for demonstration purposes, but really this should be solved
 * at the skill and agent levels (easy to spot because this thread is
 * also hooked in at the skill hook).
 *
 * @author Tim Niemueller
 */

/** Constructor. */
RoombaJoystickThread::RoombaJoystickThread()
  : Thread("RoombaJoy", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
}


void
RoombaJoystickThread::init()
{
  __joy_if = NULL;
  __roomba500_if = NULL;

  try {
    __roomba500_if = blackboard->open_for_reading<Roomba500Interface>("Roomba 500");
    __joy_if = blackboard->open_for_reading<JoystickInterface>("Joystick");

  } catch (Exception &e) {
    blackboard->close(__roomba500_if);
    blackboard->close(__joy_if);
    throw;
  }

  __last_velo = 250;
  __main_brush_enabled = false;
  __side_brush_enabled = false;
  __vacuuming_enabled  = false;
}

void
RoombaJoystickThread::finalize()
{
  blackboard->close(__roomba500_if);
  blackboard->close(__joy_if);
}


void
RoombaJoystickThread::loop()
{
  __joy_if->read();
  if (__joy_if->changed()) {
    if (__joy_if->num_axes() == 0) {
      logger->log_debug(name(), "Joystick disconnected, stopping");
      stop();
    } else if (__joy_if->pressed_buttons()) {

      bool motor_state = false;

      if (__joy_if->pressed_buttons() & JoystickInterface::BUTTON_1) {
	motor_state = true;
	__main_brush_enabled = ! __main_brush_enabled;
      }

      if (__joy_if->pressed_buttons() & JoystickInterface::BUTTON_2) {
	motor_state = true;
	__side_brush_enabled = ! __side_brush_enabled;
      }

      if (__joy_if->pressed_buttons() & JoystickInterface::BUTTON_3) {
	motor_state = true;
	__vacuuming_enabled  = ! __vacuuming_enabled;
      }

      if (motor_state) {
	Roomba500Interface::SetMotorsMessage *sm =
	  new Roomba500Interface::SetMotorsMessage(
	    __vacuuming_enabled,
	    __main_brush_enabled ? Roomba500Interface::BRUSHSTATE_FORWARD
			         : Roomba500Interface::BRUSHSTATE_OFF,
	    __side_brush_enabled ? Roomba500Interface::BRUSHSTATE_FORWARD
			         : Roomba500Interface::BRUSHSTATE_OFF
	  );
	__roomba500_if->msgq_enqueue(sm);
      }

      if (__joy_if->pressed_buttons() & JoystickInterface::BUTTON_10) {
	Roomba500Interface::DockMessage *dm =
	  new Roomba500Interface::DockMessage();
	__roomba500_if->msgq_enqueue(dm);
      }

      if (__joy_if->pressed_buttons() & JoystickInterface::BUTTON_8) {
	Roomba500Interface::SetModeMessage *sm =
	  new Roomba500Interface::SetModeMessage();

	__roomba500_if->read();
	switch (__roomba500_if->mode()) {
	case Roomba500Interface::MODE_PASSIVE:
	  sm->set_mode(Roomba500Interface::MODE_SAFE); break;
	case Roomba500Interface::MODE_SAFE:
	  sm->set_mode(Roomba500Interface::MODE_FULL); break;
	case Roomba500Interface::MODE_FULL:
	  sm->set_mode(Roomba500Interface::MODE_PASSIVE); break;
	default:
	  sm->set_mode(Roomba500Interface::MODE_PASSIVE); break;
	}
	__roomba500_if->msgq_enqueue(sm);
      }


    } else if (__joy_if->axis_x(0) == 0 && __joy_if->axis_y(0) == 0) {
      stop();
    } else {
      float velocity = __joy_if->axis_x(0) *  500;
      float radius   = __joy_if->axis_y(0) * 2000;
      if (__joy_if->num_axes() > 1) {
	velocity *= __joy_if->axis_x(1);
      }

      int16_t velmm = roundf(velocity);
      int16_t radmm = roundf(radius);
      // special case handling for "turn on place"
      if (radmm == 2000) {
	velmm =  abs(__last_velo);
	radmm =    1;
      } else if (radmm == -2000) {
	velmm =  abs(__last_velo);
	radmm = -  1;
      }

      logger->log_debug(name(), "Joystick (%f,%f,%f)  Velo %f/%i  Radius %f/%i",
			__joy_if->axis_x(0), __joy_if->axis_y(0),
			__joy_if->axis_x(1), velocity, velmm, radius, radmm);


      __last_velo = velmm;

      Roomba500Interface::DriveMessage *dm =
	new Roomba500Interface::DriveMessage(velmm, radmm);
      __roomba500_if->msgq_enqueue(dm);
    }
  }
}


void
RoombaJoystickThread::stop()
{
  Roomba500Interface::StopMessage *sm = new Roomba500Interface::StopMessage();
  __roomba500_if->msgq_enqueue(sm);
}
