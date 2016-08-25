
/***************************************************************************
 *  joystick_teleop_thread.h - Joystick teleop thread
 *
 *  Created: Sun Nov 13 23:22:29 2011 (as part of the robotino plugin)
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

#ifndef __PLUGINS_JOYSTICK_JOYSTICK_TELEOP_THREAD_H_
#define __PLUGINS_JOYSTICK_JOYSTICK_TELEOP_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <string>
#include <cfloat>

namespace fawkes {
  class MotorInterface;
  class JoystickInterface;
  class Laser360Interface;
}

class JoystickTeleOpThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  JoystickTeleOpThread();

  virtual void init();
  virtual void loop();
  virtual bool prepare_finalize_user();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void stop();
  bool is_area_free(float theta);
  void send_transrot(float vx, float vy, float omega);

 private:
  fawkes::MotorInterface     *motor_if_;
  fawkes::JoystickInterface  *joystick_if_;
  fawkes::Laser360Interface  *laser_if_;

  unsigned int cfg_axis_forward_;
  unsigned int cfg_axis_sideward_;
  unsigned int cfg_axis_rotation_;
  float        cfg_axis_threshold_;
  bool         cfg_deadman_use_axis_;
  unsigned int cfg_deadman_axis_;
  float        cfg_deadman_ax_thresh_;
  unsigned int cfg_deadman_butmask_;
  bool         cfg_drive_mode_use_axis_;
  unsigned int cfg_drive_mode_axis_;
  float        cfg_drive_mode_ax_thresh_;
  unsigned int cfg_drive_mode_butmask_;
  float        cfg_normal_max_vx_;
  float        cfg_normal_max_vy_;
  float        cfg_normal_max_omega_;
  float        cfg_special_max_vx_;
  float        cfg_special_max_vy_;
  float        cfg_special_max_omega_;
  bool         cfg_collision_safety_;
  float        cfg_collision_safety_distance_;
  unsigned int cfg_collision_safety_angle_;
  std::string  cfg_ifid_motor_;
  std::string  cfg_ifid_joystick_;
  bool         cfg_use_laser_;
  std::string  cfg_ifid_laser_;
  bool         cfg_use_ff_;
  unsigned int cfg_runstop_enable_buttons_;
  unsigned int cfg_runstop_disable_buttons_;
  
  bool         stopped_;
  float        min_distance_;

  bool         ff_weak_;
  bool         ff_strong_;

  bool         runstop_pressed_;
};


#endif
