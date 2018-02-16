
/***************************************************************************
 *  joystick_teleop_thread.cpp - Joystick teleop thread
 *
 *  Created: Sun Nov 13 16:07:40 2011 (as part of the robotino plugin)
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

#include "joystick_teleop_thread.h"

#include <interfaces/MotorInterface.h>
#include <interfaces/JoystickInterface.h>
#include <interfaces/Laser360Interface.h>
#include <utils/math/angle.h>
#include <utils/math/coord.h>


#include <cmath>

#define CFG_PREFIX "/hardware/joystick/teleop/"
#define CFG_AXIS_FORWARD   CFG_PREFIX"axis_forward"
#define CFG_AXIS_SIDEWARD  CFG_PREFIX"axis_sideward"
#define CFG_AXIS_ROTATION  CFG_PREFIX"axis_rotation"

using namespace fawkes;

/** @class JoystickTeleOpThread "joystick_teleop_thread.h"
 * Remotely control a robot using a joystick.
 * @author Tim Niemueller
 */

/** Constructor. */
JoystickTeleOpThread::JoystickTeleOpThread()
  : Thread("JoystickTeleOpThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
}


void
JoystickTeleOpThread::init()
{
  cfg_axis_forward_   = config->get_uint(CFG_PREFIX"axis_forward");
  cfg_axis_sideward_  = config->get_uint(CFG_PREFIX"axis_sideward");
  cfg_axis_rotation_  = config->get_uint(CFG_PREFIX"axis_rotation");
  cfg_axis_threshold_ = config->get_float(CFG_PREFIX"axis_threshold");

  cfg_deadman_use_axis_ = false;
  try {
    cfg_deadman_axis_      = config->get_uint(CFG_PREFIX"deadman_axis");
    cfg_deadman_ax_thresh_ = config->get_float(CFG_PREFIX"deadman_axis_threshold");
    cfg_deadman_use_axis_  = true;
  } catch (Exception &e) {
    logger->log_debug(name(), "No deadman axis configured, ignoring");
  }
  cfg_deadman_butmask_   = config->get_uint(CFG_PREFIX"deadman_button_mask");

  cfg_drive_mode_use_axis_ = false;
  try {
    cfg_drive_mode_axis_      = config->get_uint(CFG_PREFIX"drive_mode_axis");
    cfg_drive_mode_ax_thresh_ = config->get_float(CFG_PREFIX"drive_mode_axis_threshold");
    cfg_drive_mode_use_axis_  = true;
  } catch (Exception &e) {
    logger->log_debug(name(), "No drive_mode axis configured, ignoring");
  }
  cfg_drive_mode_butmask_   = config->get_uint(CFG_PREFIX"drive_mode_button_mask");

  cfg_normal_max_vx_     = config->get_float(CFG_PREFIX"drive_modes/normal/max_vx");
  cfg_normal_max_vy_     = config->get_float(CFG_PREFIX"drive_modes/normal/max_vy");
  cfg_normal_max_omega_  = config->get_float(CFG_PREFIX"drive_modes/normal/max_omega");

  cfg_special_max_vx_    = config->get_float(CFG_PREFIX"drive_modes/special/max_vx");
  cfg_special_max_vy_    = config->get_float(CFG_PREFIX"drive_modes/special/max_vy");
  cfg_special_max_omega_ = config->get_float(CFG_PREFIX"drive_modes/special/max_omega");

  cfg_collision_safety_          = config->get_bool(CFG_PREFIX"collision_safety/enabled");
  cfg_collision_safety_distance_ = config->get_float(CFG_PREFIX"collision_safety/distance");
  cfg_collision_safety_angle_    = config->get_uint(CFG_PREFIX"collision_safety/angle");

  cfg_runstop_enable_buttons_  = config->get_uint(CFG_PREFIX"runstop-enable-buttons");
  cfg_runstop_disable_buttons_ = config->get_uint(CFG_PREFIX"runstop-disable-buttons");

  cfg_ifid_motor_        = config->get_string(CFG_PREFIX"motor_interface_id");
  motor_if_ = blackboard->open_for_reading<MotorInterface>(cfg_ifid_motor_.c_str());

  cfg_ifid_joystick_     = config->get_string(CFG_PREFIX"joystick_interface_id");
  joystick_if_ =
    blackboard->open_for_reading<JoystickInterface>(cfg_ifid_joystick_.c_str());

  cfg_use_laser_ = false;
  laser_if_ = NULL;
  if (cfg_collision_safety_) {
    try {
      cfg_ifid_laser_        = config->get_string(CFG_PREFIX"laser_interface_id");
      laser_if_ =
        blackboard->open_for_reading<Laser360Interface>(cfg_ifid_laser_.c_str());
      cfg_use_laser_ = true;
    } catch (Exception &e) {
      logger->log_warn(name(), "No laser_interface_id configured, ignoring");
    }

    cfg_use_ff_ = false;
    ff_weak_ = false;
    ff_strong_ = false;
    try {
	    cfg_use_ff_ = config->get_bool(CFG_PREFIX"collision_safety/use-force-feedback");
    } catch (Exception &e) {} // ignore, use default
    logger->log_debug(name(), "Collision safety force feedback %sabled", cfg_use_ff_ ? "En" : "Dis");
    if (cfg_use_ff_) {
	    JoystickInterface::StartRumbleMessage *msg =
		    new JoystickInterface::StartRumbleMessage();

	    msg->set_strong_magnitude(0xFFFF);
	    msg->set_weak_magnitude(0x8000);
	    msg->set_length(1000);

	    joystick_if_->msgq_enqueue(msg);

    }
  } else {
    logger->log_warn(name(), "Collision safety for joystick is disabled.");
  }

  runstop_pressed_ = false;
  stopped_ = false;
}


bool
JoystickTeleOpThread::prepare_finalize_user()
{
  stop();
  return true;
}

void
JoystickTeleOpThread::finalize()
{
  blackboard->close(motor_if_);
  blackboard->close(joystick_if_);
  blackboard->close(laser_if_);
}

void
JoystickTeleOpThread::send_transrot(float vx, float vy, float omega)
{
  if (! motor_if_->has_writer())  return;

  try {
    MotorInterface::TransRotMessage *msg =
      new MotorInterface::TransRotMessage(vx, vy, omega);
    motor_if_->msgq_enqueue(msg);
    stopped_ = false;
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to send transrot: %s", e.what_no_backtrace());
  }
}


void
JoystickTeleOpThread::stop()
{
  if (! motor_if_->has_writer())  return;

  send_transrot(0., 0., 0.);
  stopped_ = true;
}

bool
JoystickTeleOpThread::is_area_free(float theta)
{
	if (! laser_if_)  return true;

  min_distance_ = FLT_MAX;
  for (int i = (-1)*cfg_collision_safety_angle_; i <= (int)cfg_collision_safety_angle_; ++i)
  {
    int angle = ((int)theta) + i;
    if (angle < 0)
    {
      angle = angle + 359;
    }
    if (laser_if_->distances(angle) > 0. && laser_if_->distances(angle) < min_distance_)
    {
      min_distance_ = laser_if_->distances(angle);
    }
    if (laser_if_->distances(angle) > 0. && laser_if_->distances(angle) < cfg_collision_safety_distance_)
    {
      return false;
    }
  }

  return true;
}

void
JoystickTeleOpThread::loop()
{
  joystick_if_->read();
  if (laser_if_)  laser_if_->read();
  motor_if_->read();

  if ((! joystick_if_->has_writer() || joystick_if_->num_axes() == 0) && ! stopped_) {
    logger->log_warn(name(), "Joystick disconnected, stopping");
    stop();
  } else if ((cfg_axis_forward_ > joystick_if_->num_axes() ||
                cfg_axis_sideward_ > joystick_if_->num_axes() ||
	              cfg_axis_rotation_ > joystick_if_->num_axes() ||
	              (cfg_deadman_use_axis_ && cfg_deadman_axis_ > joystick_if_->num_axes()))
	            && ! stopped_)
  {
    logger->log_warn(name(), "Axis number out of range, stopping");
    stop();
  } else if (joystick_if_->pressed_buttons() == cfg_runstop_enable_buttons_ &&
             ! runstop_pressed_ &&
             motor_if_->motor_state() != MotorInterface::MOTOR_DISABLED)
  {
	  try {
		  stop();
		  MotorInterface::SetMotorStateMessage *msg =
			  new MotorInterface::SetMotorStateMessage(MotorInterface::MOTOR_DISABLED);
		  motor_if_->msgq_enqueue(msg);
		  runstop_pressed_ = true;
		  logger->log_warn(name(), "Runstop ENABLED");
	  } catch (Exception &e) {
		  logger->log_error(name(), "FAILED to enable runstop: %s", e.what_no_backtrace());
	  }
  } else if (joystick_if_->pressed_buttons() == cfg_runstop_disable_buttons_ &&
             ! runstop_pressed_ &&
             motor_if_->motor_state() == MotorInterface::MOTOR_DISABLED)
  {
	  try {
		  stop();
		  MotorInterface::SetMotorStateMessage *msg =
			  new MotorInterface::SetMotorStateMessage(MotorInterface::MOTOR_ENABLED);
		  motor_if_->msgq_enqueue(msg);
		  logger->log_warn(name(), "Runstop DISABLED");
		  runstop_pressed_ = true;
	  } catch (Exception &e) {
		  logger->log_error(name(), "FAILED to enable runstop: %s", e.what_no_backtrace());
	  }
  } else if ((joystick_if_->pressed_buttons() & cfg_deadman_butmask_) ||
	     (cfg_deadman_use_axis_ &&
	      ((cfg_deadman_ax_thresh_ >= 0 && joystick_if_->axis(cfg_deadman_axis_) > cfg_deadman_ax_thresh_) ||
	       (cfg_deadman_ax_thresh_ <  0 && joystick_if_->axis(cfg_deadman_axis_) < cfg_deadman_ax_thresh_))))
  {
	  runstop_pressed_ = false;
    if (fabsf(joystick_if_->axis(cfg_axis_forward_)) < cfg_axis_threshold_ &&
         fabsf(joystick_if_->axis(cfg_axis_sideward_)) < cfg_axis_threshold_ &&
         fabsf(joystick_if_->axis(cfg_axis_rotation_)) < cfg_axis_threshold_) {
      stop();
    } else {
      float vx = 0, vy = 0, omega = 0;

      if ((joystick_if_->pressed_buttons() & cfg_drive_mode_butmask_) ||
        (cfg_drive_mode_use_axis_ &&
          ((cfg_drive_mode_ax_thresh_ >= 0 &&
            joystick_if_->axis(cfg_drive_mode_axis_) > cfg_drive_mode_ax_thresh_) ||
          (cfg_drive_mode_ax_thresh_ <  0 &&
          joystick_if_->axis(cfg_drive_mode_axis_) < cfg_drive_mode_ax_thresh_))))
      {
	      if (fabsf(joystick_if_->axis(cfg_axis_forward_)) > cfg_axis_threshold_) {
		      vx    = joystick_if_->axis(cfg_axis_forward_) * cfg_special_max_vx_;
	      }
	      if (fabsf(joystick_if_->axis(cfg_axis_sideward_)) > cfg_axis_threshold_) {
		      vy    = joystick_if_->axis(cfg_axis_sideward_) * cfg_special_max_vy_;
	      }
	      if (fabsf(joystick_if_->axis(cfg_axis_rotation_)) > cfg_axis_threshold_) {
		      omega = joystick_if_->axis(cfg_axis_rotation_) * cfg_special_max_omega_;
	      }
      } else {
	      if (fabsf(joystick_if_->axis(cfg_axis_forward_)) > cfg_axis_threshold_) {
		      vx    = joystick_if_->axis(cfg_axis_forward_) * cfg_normal_max_vx_;
	      }
	      if (fabsf(joystick_if_->axis(cfg_axis_sideward_)) > cfg_axis_threshold_) {
		      vy    = joystick_if_->axis(cfg_axis_sideward_) * cfg_normal_max_vy_;
	      }
	      if (fabsf(joystick_if_->axis(cfg_axis_rotation_)) > cfg_axis_threshold_) {
		      omega = joystick_if_->axis(cfg_axis_rotation_) * cfg_normal_max_omega_;
	      }
      }

      float theta, distance;
      cart2polar2d(vx, vy, &theta, &distance);
      bool area_free = is_area_free(rad2deg(theta));
      if (!cfg_use_laser_ || area_free) // if we have no laser or area is free, move
      {  
        if (laser_if_ && laser_if_->has_writer() && min_distance_ < 2*cfg_collision_safety_distance_)
        {
          logger->log_warn(name(),"slow down");
          vx = vx * min_distance_ / 2 / cfg_collision_safety_distance_;
          vy = vy * min_distance_ / 2 / cfg_collision_safety_distance_;

          if (cfg_use_ff_ && ! ff_weak_ && joystick_if_->supported_ff_effects() != 0) {
	          JoystickInterface::StartRumbleMessage *msg =
		          new JoystickInterface::StartRumbleMessage();

	          msg->set_strong_magnitude(0xFFFF);
	          msg->set_weak_magnitude(0x8000);

	          joystick_if_->msgq_enqueue(msg);
	          ff_weak_ = true;
	          ff_strong_ = false;
          }
        } else if (ff_weak_ || ff_strong_) {
	        JoystickInterface::StopRumbleMessage *msg =
		        new JoystickInterface::StopRumbleMessage();

	        joystick_if_->msgq_enqueue(msg);
	        ff_weak_ = false;
	        ff_strong_ = false;
        }
        send_transrot(vx, vy, omega);
        runstop_pressed_ = false;
      }
      else if (cfg_use_laser_ && ! area_free)
      {
        logger->log_warn(name(),"obstacle reached");
        send_transrot(0.,0.,omega);

        if (cfg_use_ff_ && ! ff_weak_ && joystick_if_->supported_ff_effects() != 0) {
	        JoystickInterface::StartRumbleMessage *msg =
		        new JoystickInterface::StartRumbleMessage();

	        msg->set_strong_magnitude(0x8000);
	        msg->set_weak_magnitude(0xFFFF);
      
	        logger->log_debug(name(), "Enabling strong rumble");
	        joystick_if_->msgq_enqueue(msg);
	        ff_strong_ = true;
	        ff_weak_ = false;
        }
      }
    }
  } else if (! stopped_) {
    runstop_pressed_ = false;
    stop();
  } else if (joystick_if_->pressed_buttons() != cfg_runstop_enable_buttons_ &&
             joystick_if_->pressed_buttons() != cfg_runstop_enable_buttons_)
  {
	  runstop_pressed_ = false;
  }
}
