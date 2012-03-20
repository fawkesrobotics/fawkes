
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

#define CFG_PREFIX "/hardware/roomba/joystick/"
#define CFG_BUT_MAIN_BRUSH CFG_PREFIX"but_main_brush"
#define CFG_BUT_SIDE_BRUSH CFG_PREFIX"but_side_brush"
#define CFG_BUT_VACUUMING  CFG_PREFIX"but_vacuuming"
#define CFG_BUT_DOCK       CFG_PREFIX"but_dock"
#define CFG_BUT_SPOT       CFG_PREFIX"but_spot"
#define CFG_BUT_MODE       CFG_PREFIX"but_mode"
#define CFG_AXIS_FORWARD   CFG_PREFIX"axis_forward"
#define CFG_AXIS_SIDEWARD  CFG_PREFIX"axis_sideward"
#define CFG_AXIS_SPEED     CFG_PREFIX"axis_speed"

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

  __cfg_but_main_brush = confval(CFG_BUT_MAIN_BRUSH, JoystickInterface::BUTTON_1);
  __cfg_but_side_brush = confval(CFG_BUT_SIDE_BRUSH, JoystickInterface::BUTTON_2);
  __cfg_but_vacuuming  = confval(CFG_BUT_VACUUMING,  JoystickInterface::BUTTON_3);
  __cfg_but_dock       = confval(CFG_BUT_DOCK,       JoystickInterface::BUTTON_4);
  __cfg_but_spot       = confval(CFG_BUT_SPOT,       JoystickInterface::BUTTON_5);
  __cfg_but_mode       = confval(CFG_BUT_MODE,       JoystickInterface::BUTTON_6);

  __cfg_axis_forward   = confval(CFG_AXIS_FORWARD,  0);
  __cfg_axis_sideward  = confval(CFG_AXIS_SIDEWARD, 1);
  __cfg_axis_speed     = confval(CFG_AXIS_SPEED,    2);

  __cfg_min_radius     = config->get_uint(CFG_PREFIX"min_radius");
  __cfg_max_radius     = config->get_uint(CFG_PREFIX"max_radius");
  __cfg_max_velocity   = config->get_uint(CFG_PREFIX"max_velocity");

  try {
    __roomba500_if = blackboard->open_for_reading<Roomba500Interface>("Roomba 500");
    __joy_if = blackboard->open_for_reading<JoystickInterface>("Joystick");

  } catch (Exception &e) {
    blackboard->close(__roomba500_if);
    blackboard->close(__joy_if);
    throw;
  }

  if (__cfg_axis_forward > __joy_if->maxlenof_axis()) {
    throw Exception("Invalid forward axis value %u, must be smaller than %u",
		    __cfg_axis_forward, __joy_if->maxlenof_axis());
  }
  if (__cfg_axis_sideward > __joy_if->maxlenof_axis()) {
    throw Exception("Invalid sideward axis value %u, must be smaller than %u",
		    __cfg_axis_sideward, __joy_if->maxlenof_axis());
  }
  if (__cfg_axis_speed > __joy_if->maxlenof_axis()) {
    logger->log_warn(name(), "Speed axis disabled, setting half max speed.");
  }

  __last_velo = __cfg_max_velocity / 2;
  __main_brush_enabled = false;
  __side_brush_enabled = false;
  __vacuuming_enabled  = false;

  __strong_rumble = false;
  __weak_rumble   = false;
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
  __roomba500_if->read();

  if (__joy_if->supported_ff_effects() & JoystickInterface::JFF_RUMBLE) {
    uint16_t mlb = __roomba500_if->light_bump_left();
    mlb = std::max(mlb, __roomba500_if->light_bump_front_left());
    mlb = std::max(mlb, __roomba500_if->light_bump_center_left());
    mlb = std::max(mlb, __roomba500_if->light_bump_center_right());
    mlb = std::max(mlb, __roomba500_if->light_bump_front_right());
    mlb = std::max(mlb, __roomba500_if->light_bump_right());

    if (__roomba500_if->is_bump_left() || __roomba500_if->is_bump_right()) {
      if (! __weak_rumble) {
	JoystickInterface::StartRumbleMessage *msg =
	  new JoystickInterface::StartRumbleMessage();

	msg->set_strong_magnitude(0xFFFF);
	msg->set_weak_magnitude(0x8000);
      
	__joy_if->msgq_enqueue(msg);
	__weak_rumble = true;
	__strong_rumble = false;
      }
    } else if ((mlb > 200) && !__strong_rumble) {
      JoystickInterface::StartRumbleMessage *msg =
	new JoystickInterface::StartRumbleMessage();

      float mf = (mlb / 1000.f);
      if (mf > 1)   mf = 1;
      if (mf < 0.4) mf = 0.4;

      msg->set_weak_magnitude((uint16_t)floorf(mf * 0xFFFF));
      if (mf > 0.8) msg->set_strong_magnitude(0x8000);

      __joy_if->msgq_enqueue(msg);

      __weak_rumble = false;
      __strong_rumble = true;
    } else if (__weak_rumble || __strong_rumble) {
      JoystickInterface::StopRumbleMessage *msg =
	new JoystickInterface::StopRumbleMessage();
      __joy_if->msgq_enqueue(msg);
      
      __weak_rumble = __strong_rumble = false;
    }
  }

  if (__joy_if->changed()) {
    if (__joy_if->num_axes() == 0) {
      logger->log_debug(name(), "Joystick disconnected, stopping");
      stop();
    } else if (__joy_if->pressed_buttons()) {

      bool motor_state = false;

      if (__joy_if->pressed_buttons() & __cfg_but_main_brush) {
	motor_state = true;
	__main_brush_enabled = ! __main_brush_enabled;
      }

      if (__joy_if->pressed_buttons() & __cfg_but_side_brush) {
	motor_state = true;
	__side_brush_enabled = ! __side_brush_enabled;
      }

      if (__joy_if->pressed_buttons() & __cfg_but_vacuuming) {
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

      if (__joy_if->pressed_buttons() & __cfg_but_dock) {
	Roomba500Interface::DockMessage *dm =
	  new Roomba500Interface::DockMessage();
	__roomba500_if->msgq_enqueue(dm);
      }

      if (__joy_if->pressed_buttons() & __cfg_but_spot) {
	/*
	Roomba500Interface::DockMessage *dm =
	  new Roomba500Interface::DockMessage();
	__roomba500_if->msgq_enqueue(dm);
	*/
      }

      if (__joy_if->pressed_buttons() & __cfg_but_mode) {
	Roomba500Interface::SetModeMessage *sm =
	  new Roomba500Interface::SetModeMessage();

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


    } else if (__joy_if->axis(__cfg_axis_forward) == 0 &&
	       __joy_if->axis(__cfg_axis_sideward) == 0) {
      stop();
    } else {
      float forward  = __joy_if->axis(__cfg_axis_forward) *  __cfg_max_velocity;
      float sideward = __joy_if->axis(__cfg_axis_sideward);
      float radius   = copysignf(std::max(__cfg_min_radius,
					  (int)(1. - fabs(sideward)) *
					  __cfg_max_radius),
				 sideward);
      float velocity = .5;
      if (__cfg_axis_speed < __joy_if->maxlenof_axis()) {
	velocity = __joy_if->axis(__cfg_axis_speed);
      }

      int16_t velmm = (int16_t)roundf(forward * velocity);
      int16_t radmm = (int16_t)roundf(radius);
      // special case handling for "turn on place"
      if (fabsf(__joy_if->axis(__cfg_axis_forward)) < 0.1) {
	velmm =  (int16_t)fabs(sideward * velocity) * __cfg_max_velocity;
	radmm =  (int16_t)copysignf(1, sideward);
      }

      /*
      logger->log_debug(name(), "Joystick (%f,%f,%f)  Velo %f/%i  Radius %f/%i",
			__joy_if->axis(__cfg_axis_forward),
			__joy_if->axis(__cfg_axis_sideward),
			__joy_if->axis(__cfg_axis_speed),
			velocity, velmm, radius, radmm);
      */

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


unsigned int
RoombaJoystickThread::confval(const char *path, unsigned int default_value)
{
  try {
    return config->get_uint(path);
  } catch (Exception &e) {
    return default_value;
  }

}
