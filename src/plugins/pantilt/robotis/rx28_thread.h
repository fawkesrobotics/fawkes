
/***************************************************************************
 *  rx28_thread.h - RX28 pan/tilt unit act thread
 *
 *  Created: Thu Jun 18 09:52:16 2009
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

#ifndef __PLUGINS_PANTILT_ROBOTIS_RX28_THREAD_H_
#define __PLUGINS_PANTILT_ROBOTIS_RX28_THREAD_H_

#include "../act_thread.h"

#ifdef HAVE_TF
#  include <aspect/tf.h>
#endif
#include <blackboard/interface_listener.h>
#include <utils/time/time.h>

#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <string>
#include <memory>

namespace fawkes {
  class PanTiltInterface;
  class LedInterface;
  class JointInterface;
  class ReadWriteLock;
  class WaitCondition;
}

class RobotisRX28;

class PanTiltRX28Thread
: public PanTiltActThread,
#ifdef HAVE_TF
  public fawkes::TransformAspect,
#endif
  public fawkes::BlackBoardInterfaceListener
{
 public:
  PanTiltRX28Thread(std::string &pantilt_cfg_prefix,
		    std::string &ptu_cfg_prefix,
		    std::string &ptu_name);

  virtual void init();
  virtual bool prepare_finalize_user();
  virtual void finalize();
  virtual void loop();

  // For BlackBoardInterfaceListener
  virtual bool bb_interface_message_received(fawkes::Interface *interface,
					     fawkes::Message *message) throw();

  void update_sensor_values();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::PanTiltInterface *__pantilt_if;
  fawkes::LedInterface     *__led_if;
  fawkes::JointInterface   *__panjoint_if;
  fawkes::JointInterface   *__tiltjoint_if;

  fawkes::RefPtr<RobotisRX28> __rx28;

  std::string  __pantilt_cfg_prefix;
  std::string  __ptu_cfg_prefix;
  std::string  __ptu_name;
  std::string  __cfg_device;
  unsigned int __cfg_read_timeout_ms;
  unsigned int __cfg_disc_timeout_ms;
  unsigned int __cfg_pan_servo_id;
  unsigned int __cfg_tilt_servo_id;
  bool         __cfg_goto_zero_start;
  bool         __cfg_turn_off;
  unsigned int __cfg_cw_compl_margin;
  unsigned int __cfg_ccw_compl_margin;
  unsigned int __cfg_cw_compl_slope;
  unsigned int __cfg_ccw_compl_slope;
  float        __cfg_pan_min;
  float        __cfg_pan_max;
  float        __cfg_tilt_min;
  float        __cfg_tilt_max;
  float        __cfg_pan_margin;
  float        __cfg_tilt_margin;
  float        __cfg_pan_offset;
  float        __cfg_tilt_offset;
  float        __cfg_pan_start;
  float        __cfg_tilt_start;
#ifdef HAVE_TF
  std::string  __cfg_base_frame;
  std::string  __cfg_pan_link;
  std::string  __cfg_tilt_link;

  fawkes::tf::Vector3  __translation_pan;
  fawkes::tf::Vector3  __translation_tilt;

  bool         __cfg_publish_transforms;
#endif

  float         __last_pan;
  float         __last_tilt;

  class WorkerThread : public fawkes::Thread
  {
  public:
    WorkerThread(std::string ptu_name, fawkes::Logger *logger,
		 fawkes::RefPtr<RobotisRX28> rx28,
		 unsigned char pan_servo_id, unsigned char tilt_servo_id,
		 float &pan_min, float &pan_max, float &tilt_min, float &tilt_max,
		 float &pan_offset, float &tilt_offset);

    ~WorkerThread();
    void goto_pantilt(float pan, float tilt);
    void goto_pantilt_timed(float pan, float tilt, float time_sec);
    void get_pantilt(float &pan, float &tilt);
    void get_pantilt(float &pan, float &tilt, fawkes::Time &time);
    void set_velocities(float pan_vel, float tilt_vel);
    void get_velocities(float &pan_vel, float &tilt_vel);
    void set_margins(float pan_margin, float tilt_margin);
    bool is_final();
    bool is_enabled();
    void set_enabled(bool enabled);
    void set_led_enabled(bool enabled);
    void stop_motion();
    bool has_fresh_data();
    void wait_for_fresh_data();

    virtual void loop();

    /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
    protected: virtual void run() { Thread::run(); }

  private:
    void exec_goto_pantilt(float pan, float tilt);

  private:
    fawkes::ReadWriteLock       *__rx28_rwlock;
    fawkes::RefPtr<RobotisRX28>  __rx28;
    fawkes::Logger              *__logger;
    fawkes::WaitCondition       *__update_waitcond;

    unsigned char __pan_servo_id;
    unsigned char __tilt_servo_id;

    float         __pan_min;
    float         __pan_max;
    float         __tilt_min;
    float         __tilt_max;
    float         __pan_offset;
    float         __tilt_offset;
    float         __max_pan_speed;
    float         __max_tilt_speed;
    float         __pan_margin;
    float         __tilt_margin;

    fawkes::ReadWriteLock *__value_rwlock;
    bool  __move_pending;
    float __target_pan;
    float __target_tilt;
    bool  __enable;
    bool  __disable;
    bool  __velo_pending;
    unsigned int __pan_vel;
    unsigned int __tilt_vel;
    bool  __led_enable;
    bool  __led_disable;
    fawkes::Time  __pantilt_time;

    bool __fresh_data;
    fawkes::Mutex *__fresh_data_mutex;

  };

  WorkerThread *__wt;
};

#endif
