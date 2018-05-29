
/***************************************************************************
 *  driver_thread.h - Robotis dynamixel servo driver thread
 *
 *  Created: Mon Mar 23 20:26:52 2015
 *  Copyright  2006-2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_DYNAMIXEL_DRIVER_THREAD_H_
#define __PLUGINS_DYNAMIXEL_DRIVER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <core/threading/scoped_rwlock.h>
#include <core/threading/read_write_lock.h>

#include <blackboard/interface_listener.h>
#include <utils/time/time.h>

#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <string>
#include <memory>

namespace fawkes {
  class DynamixelServoInterface;
  class LedInterface;
  class JointInterface;
}

class DynamixelChain;

class Dynamixel;

class DynamixelDriverThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  DynamixelDriverThread(std::string &cfg_name, std::string &cfg_prefix);

  virtual void init();
  virtual void finalize();
  virtual void loop();

  // For BlackBoardInterfaceListener
  virtual bool bb_interface_message_received(fawkes::Interface *interface,
					     fawkes::Message *message) throw();

  void exec_sensor();
  void exec_act();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:

  class Servo {
   public:
    fawkes::DynamixelServoInterface  *servo_if;
    fawkes::LedInterface             *led_if;
    fawkes::JointInterface           *joint_if;

    fawkes::ReadWriteLock *value_rwlock;

    bool         move_pending;
    float        target_angle;
    bool         enable;
    bool         disable;
    bool         velo_pending;
    unsigned int vel;
    bool         mode_set_pending;
    bool         recover_pending;
    unsigned int new_mode;
    bool         led_enable;
    bool         led_disable;
    float        max_speed;
    float        angle_margin;
    unsigned int torque_limit;
    fawkes::Time time;
    float        last_angle;
  };
  std::map<unsigned int, Servo> servos_;

  fawkes::RefPtr<DynamixelChain>  chain_;
  fawkes::ReadWriteLock          *chain_rwlock_;

  std::string   cfg_prefix_;
  std::string   cfg_name_;
  std::string   cfg_device_;
  unsigned int  cfg_read_timeout_ms_;
  unsigned int  cfg_disc_timeout_ms_;
  bool          cfg_goto_zero_start_;
  bool          cfg_turn_off_;
  unsigned int  cfg_cw_compl_margin_;
  unsigned int  cfg_ccw_compl_margin_;
  unsigned int  cfg_cw_compl_slope_;
  unsigned int  cfg_ccw_compl_slope_;
  float         cfg_def_angle_margin_;
  bool          cfg_enable_echo_fix_;
  bool          cfg_enable_connection_stability_;
  bool          cfg_autorecover_enabled_;
  unsigned char cfg_autorecover_flags_;
  float         cfg_torque_limit_;
  unsigned char cfg_temperature_limit_;
  bool          cfg_prevent_alarm_shutdown_;
  float         cfg_prevent_alarm_shutdown_threshold_;
  float         cfg_min_voltage_;
  float         cfg_max_voltage_;
  std::vector<unsigned int> cfg_servos_to_discover_;
  bool          cfg_enable_verbose_output_;
  
  void  goto_angle(unsigned int servo_id, float angle);
  void  goto_angle_timed(unsigned int servo_id, float angle, float time_sec);
  float get_angle(unsigned int servo_id);
  float get_angle(unsigned int servo_id, fawkes::Time &time);
  void  set_velocity(unsigned int servo_id, float vel);
  float get_velocity(unsigned int servo_id);
  void  set_speed(unsigned int servo_id, unsigned int speed);
  void  set_mode(unsigned int servo_id, unsigned int new_mode);
  void  set_margin(unsigned int servo_id, float margin);
  bool  is_final(unsigned int servo_id);
  bool  is_enabled(unsigned int servo_id);
  void  set_enabled(unsigned int servo_id, bool enabled);
  void  set_led_enabled(unsigned int servo_id, bool enabled);
  void  stop_motion(unsigned int servo_id);
  bool  has_fresh_data();
  void  wait_for_fresh_data();

  void exec_goto_angle(unsigned int servo_id, float angle);
  void exec_set_mode(unsigned int servo_id, unsigned int new_mode);

 private:
  fawkes::WaitCondition       *update_waitcond_;

  bool fresh_data_;
  fawkes::Mutex *fresh_data_mutex_;
};

#endif
