
/***************************************************************************
 *  joystick_thread.cpp - Robotino joystick thread
 *
 *  Created: Sun Nov 13 16:07:40 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include "joystick_thread.h"

#include <interfaces/MotorInterface.h>
#include <interfaces/JoystickInterface.h>

#include <cmath>

#define CFG_PREFIX "/hardware/robotino/joystick/"
#define CFG_AXIS_FORWARD   CFG_PREFIX"axis_forward"
#define CFG_AXIS_SIDEWARD  CFG_PREFIX"axis_sideward"
#define CFG_AXIS_ROTATION  CFG_PREFIX"axis_rotation"

using namespace fawkes;

/** @class RobotinoJoystickThread "joystick_thread.h"
 * Robotino act hook integration thread.
 * This thread integrates into the Fawkes main loop at the ACT hook and
 * executes motion commands.
 * @author Tim Niemueller
 */

/** Constructor. */
RobotinoJoystickThread::RobotinoJoystickThread()
  : Thread("RobotinoJoystickThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
}


void
RobotinoJoystickThread::init()
{
  cfg_axis_forward_   = config->get_uint(CFG_AXIS_FORWARD);
  cfg_axis_sideward_  = config->get_uint(CFG_AXIS_SIDEWARD);
  cfg_axis_rotation_  = config->get_uint(CFG_AXIS_ROTATION);

  cfg_max_vx_    = config->get_float(CFG_PREFIX"max_vx");
  cfg_max_vy_    = config->get_float(CFG_PREFIX"max_vy");
  cfg_max_omega_ = config->get_float(CFG_PREFIX"max_omega");

  motor_if_ = blackboard->open_for_reading<MotorInterface>("Robotino");
  joystick_if_ =
    blackboard->open_for_reading<JoystickInterface>("Joystick");
}


bool
RobotinoJoystickThread::prepare_finalize_user()
{
  stop();
  return true;
}

void
RobotinoJoystickThread::finalize()
{
  blackboard->close(motor_if_);
  blackboard->close(joystick_if_);
}

void
RobotinoJoystickThread::send_transrot(float vx, float vy, float omega)
{
  MotorInterface::TransRotMessage *msg =
    new MotorInterface::TransRotMessage(vx, vy, omega);
  motor_if_->msgq_enqueue(msg);
}


void
RobotinoJoystickThread::stop()
{
  send_transrot(0., 0., 0.);
}

void
RobotinoJoystickThread::loop()
{
  joystick_if_->read();

  if (joystick_if_->changed()) {
    if (joystick_if_->num_axes() == 0) {
      logger->log_debug(name(), "Joystick disconnected, stopping");
      stop();
    } else if (fabsf(joystick_if_->axis(cfg_axis_forward_)) < 0.2 &&
               fabsf(joystick_if_->axis(cfg_axis_sideward_)) < 0.2 &&
               fabsf(joystick_if_->axis(cfg_axis_rotation_)) < 0.2) {
      stop();
    } else {
      float vx    = joystick_if_->axis(cfg_axis_forward_) * cfg_max_vx_;
      float vy    = joystick_if_->axis(cfg_axis_sideward_) * cfg_max_vy_;
      float omega = joystick_if_->axis(cfg_axis_rotation_) * cfg_max_omega_;

      send_transrot(vx, vy, omega);
    }
  }
}
