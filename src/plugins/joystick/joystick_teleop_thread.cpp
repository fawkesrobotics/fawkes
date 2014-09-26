
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

  cfg_ifid_motor_        = config->get_string(CFG_PREFIX"motor_interface_id");
  cfg_ifid_joystick_     = config->get_string(CFG_PREFIX"joystick_interface_id");

  motor_if_ = blackboard->open_for_reading<MotorInterface>(cfg_ifid_motor_.c_str());
  joystick_if_ =
    blackboard->open_for_reading<JoystickInterface>(cfg_ifid_joystick_.c_str());

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

void
JoystickTeleOpThread::loop()
{
  joystick_if_->read();

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
  } else if ((joystick_if_->pressed_buttons() & cfg_deadman_butmask_) ||
	     (cfg_deadman_use_axis_ &&
	      ((cfg_deadman_ax_thresh_ >= 0 && joystick_if_->axis(cfg_deadman_axis_) > cfg_deadman_ax_thresh_) ||
	       (cfg_deadman_ax_thresh_ <  0 && joystick_if_->axis(cfg_deadman_axis_) < cfg_deadman_ax_thresh_))))
  {
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
	vx    = joystick_if_->axis(cfg_axis_forward_) * cfg_special_max_vx_;
	vy    = joystick_if_->axis(cfg_axis_sideward_) * cfg_special_max_vy_;
	omega = joystick_if_->axis(cfg_axis_rotation_) * cfg_special_max_omega_;
      } else {
	vx    = joystick_if_->axis(cfg_axis_forward_) * cfg_normal_max_vx_;
	vy    = joystick_if_->axis(cfg_axis_sideward_) * cfg_normal_max_vy_;
	omega = joystick_if_->axis(cfg_axis_rotation_) * cfg_normal_max_omega_;
      }

      send_transrot(vx, vy, omega);
    }
  } else if (! stopped_) {
    stop();
  }
}
