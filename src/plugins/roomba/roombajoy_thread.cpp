
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

#include <interfaces/JoystickInterface.h>
#include <interfaces/Roomba500Interface.h>

#include <cstdlib>

#define CFG_PREFIX "/hardware/roomba/joystick/"
#define CFG_BUT_MAIN_BRUSH CFG_PREFIX "but_main_brush"
#define CFG_BUT_SIDE_BRUSH CFG_PREFIX "but_side_brush"
#define CFG_BUT_VACUUMING CFG_PREFIX "but_vacuuming"
#define CFG_BUT_DOCK CFG_PREFIX "but_dock"
#define CFG_BUT_SPOT CFG_PREFIX "but_spot"
#define CFG_BUT_MODE CFG_PREFIX "but_mode"
#define CFG_AXIS_FORWARD CFG_PREFIX "axis_forward"
#define CFG_AXIS_SIDEWARD CFG_PREFIX "axis_sideward"
#define CFG_AXIS_SPEED CFG_PREFIX "axis_speed"

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
	joy_if_       = NULL;
	roomba500_if_ = NULL;

	cfg_but_main_brush_ = confval(CFG_BUT_MAIN_BRUSH, JoystickInterface::BUTTON_1);
	cfg_but_side_brush_ = confval(CFG_BUT_SIDE_BRUSH, JoystickInterface::BUTTON_2);
	cfg_but_vacuuming_  = confval(CFG_BUT_VACUUMING, JoystickInterface::BUTTON_3);
	cfg_but_dock_       = confval(CFG_BUT_DOCK, JoystickInterface::BUTTON_4);
	cfg_but_spot_       = confval(CFG_BUT_SPOT, JoystickInterface::BUTTON_5);
	cfg_but_mode_       = confval(CFG_BUT_MODE, JoystickInterface::BUTTON_6);

	cfg_axis_forward_  = confval(CFG_AXIS_FORWARD, 0);
	cfg_axis_sideward_ = confval(CFG_AXIS_SIDEWARD, 1);
	cfg_axis_speed_    = confval(CFG_AXIS_SPEED, 2);

	cfg_min_radius_   = config->get_uint(CFG_PREFIX "min_radius");
	cfg_max_radius_   = config->get_uint(CFG_PREFIX "max_radius");
	cfg_max_velocity_ = config->get_uint(CFG_PREFIX "max_velocity");

	try {
		roomba500_if_ = blackboard->open_for_reading<Roomba500Interface>("Roomba 500");
		joy_if_       = blackboard->open_for_reading<JoystickInterface>("Joystick");

	} catch (Exception &e) {
		blackboard->close(roomba500_if_);
		blackboard->close(joy_if_);
		throw;
	}

	if (cfg_axis_forward_ > joy_if_->maxlenof_axis()) {
		throw Exception("Invalid forward axis value %u, must be smaller than %u",
		                cfg_axis_forward_,
		                joy_if_->maxlenof_axis());
	}
	if (cfg_axis_sideward_ > joy_if_->maxlenof_axis()) {
		throw Exception("Invalid sideward axis value %u, must be smaller than %u",
		                cfg_axis_sideward_,
		                joy_if_->maxlenof_axis());
	}
	if (cfg_axis_speed_ > joy_if_->maxlenof_axis()) {
		logger->log_warn(name(), "Speed axis disabled, setting half max speed.");
	}

	last_velo_          = cfg_max_velocity_ / 2;
	main_brush_enabled_ = false;
	side_brush_enabled_ = false;
	vacuuming_enabled_  = false;

	strong_rumble_ = false;
	weak_rumble_   = false;
}

void
RoombaJoystickThread::finalize()
{
	blackboard->close(roomba500_if_);
	blackboard->close(joy_if_);
}

void
RoombaJoystickThread::loop()
{
	joy_if_->read();
	roomba500_if_->read();

	if (joy_if_->supported_ff_effects() & JoystickInterface::JFF_RUMBLE) {
		uint16_t mlb = roomba500_if_->light_bump_left();
		mlb          = std::max(mlb, roomba500_if_->light_bump_front_left());
		mlb          = std::max(mlb, roomba500_if_->light_bump_center_left());
		mlb          = std::max(mlb, roomba500_if_->light_bump_center_right());
		mlb          = std::max(mlb, roomba500_if_->light_bump_front_right());
		mlb          = std::max(mlb, roomba500_if_->light_bump_right());

		if (roomba500_if_->is_bump_left() || roomba500_if_->is_bump_right()) {
			if (!weak_rumble_) {
				JoystickInterface::StartRumbleMessage *msg = new JoystickInterface::StartRumbleMessage();

				msg->set_strong_magnitude(0xFFFF);
				msg->set_weak_magnitude(0x8000);

				joy_if_->msgq_enqueue(msg);
				weak_rumble_   = true;
				strong_rumble_ = false;
			}
		} else if ((mlb > 200) && !strong_rumble_) {
			JoystickInterface::StartRumbleMessage *msg = new JoystickInterface::StartRumbleMessage();

			float mf = (mlb / 1000.f);
			if (mf > 1)
				mf = 1;
			if (mf < 0.4)
				mf = 0.4;

			msg->set_weak_magnitude((uint16_t)floorf(mf * 0xFFFF));
			if (mf > 0.8)
				msg->set_strong_magnitude(0x8000);

			joy_if_->msgq_enqueue(msg);

			weak_rumble_   = false;
			strong_rumble_ = true;
		} else if (weak_rumble_ || strong_rumble_) {
			JoystickInterface::StopRumbleMessage *msg = new JoystickInterface::StopRumbleMessage();
			joy_if_->msgq_enqueue(msg);

			weak_rumble_ = strong_rumble_ = false;
		}
	}

	if (joy_if_->changed()) {
		if (joy_if_->num_axes() == 0) {
			logger->log_debug(name(), "Joystick disconnected, stopping");
			stop();
		} else if (joy_if_->pressed_buttons()) {
			bool motor_state = false;

			if (joy_if_->pressed_buttons() & cfg_but_main_brush_) {
				motor_state         = true;
				main_brush_enabled_ = !main_brush_enabled_;
			}

			if (joy_if_->pressed_buttons() & cfg_but_side_brush_) {
				motor_state         = true;
				side_brush_enabled_ = !side_brush_enabled_;
			}

			if (joy_if_->pressed_buttons() & cfg_but_vacuuming_) {
				motor_state        = true;
				vacuuming_enabled_ = !vacuuming_enabled_;
			}

			if (motor_state) {
				Roomba500Interface::SetMotorsMessage *sm =
				  new Roomba500Interface::SetMotorsMessage(vacuuming_enabled_,
				                                           main_brush_enabled_
				                                             ? Roomba500Interface::BRUSHSTATE_FORWARD
				                                             : Roomba500Interface::BRUSHSTATE_OFF,
				                                           side_brush_enabled_
				                                             ? Roomba500Interface::BRUSHSTATE_FORWARD
				                                             : Roomba500Interface::BRUSHSTATE_OFF);
				roomba500_if_->msgq_enqueue(sm);
			}

			if (joy_if_->pressed_buttons() & cfg_but_dock_) {
				Roomba500Interface::DockMessage *dm = new Roomba500Interface::DockMessage();
				roomba500_if_->msgq_enqueue(dm);
			}

			if (joy_if_->pressed_buttons() & cfg_but_spot_) {
				/*
	Roomba500Interface::DockMessage *dm =
	  new Roomba500Interface::DockMessage();
	roomba500_if_->msgq_enqueue(dm);
	*/
			}

			if (joy_if_->pressed_buttons() & cfg_but_mode_) {
				Roomba500Interface::SetModeMessage *sm = new Roomba500Interface::SetModeMessage();

				switch (roomba500_if_->mode()) {
				case Roomba500Interface::MODE_PASSIVE: sm->set_mode(Roomba500Interface::MODE_SAFE); break;
				case Roomba500Interface::MODE_SAFE: sm->set_mode(Roomba500Interface::MODE_FULL); break;
				case Roomba500Interface::MODE_FULL: sm->set_mode(Roomba500Interface::MODE_PASSIVE); break;
				default: sm->set_mode(Roomba500Interface::MODE_PASSIVE); break;
				}
				roomba500_if_->msgq_enqueue(sm);
			}

		} else if (joy_if_->axis(cfg_axis_forward_) == 0 && joy_if_->axis(cfg_axis_sideward_) == 0) {
			stop();
		} else {
			float forward  = joy_if_->axis(cfg_axis_forward_) * cfg_max_velocity_;
			float sideward = joy_if_->axis(cfg_axis_sideward_);
			float radius =
			  copysignf(std::max(cfg_min_radius_, (int)(1. - fabsf(sideward)) * cfg_max_radius_),
			            sideward);
			float velocity = .5;
			if (cfg_axis_speed_ < joy_if_->maxlenof_axis()) {
				velocity = joy_if_->axis(cfg_axis_speed_);
			}

			int16_t velmm = (int16_t)roundf(forward * velocity);
			int16_t radmm = (int16_t)roundf(radius);
			// special case handling for "turn on place"
			if (fabsf(joy_if_->axis(cfg_axis_forward_)) < 0.1) {
				velmm = (int16_t)((double)fabsf(sideward * velocity) * cfg_max_velocity_);
				radmm = (int16_t)copysignf(1, sideward);
			}

			/*
      logger->log_debug(name(), "Joystick (%f,%f,%f)  Velo %f/%i  Radius %f/%i",
			joy_if_->axis(cfg_axis_forward_),
			joy_if_->axis(cfg_axis_sideward_),
			joy_if_->axis(cfg_axis_speed_),
			velocity, velmm, radius, radmm);
      */

			last_velo_ = velmm;

			Roomba500Interface::DriveMessage *dm = new Roomba500Interface::DriveMessage(velmm, radmm);
			roomba500_if_->msgq_enqueue(dm);
		}
	}
}

void
RoombaJoystickThread::stop()
{
	Roomba500Interface::StopMessage *sm = new Roomba500Interface::StopMessage();
	roomba500_if_->msgq_enqueue(sm);
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
