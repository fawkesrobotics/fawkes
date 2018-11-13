
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

#ifndef _PLUGINS_PANTILT_ROBOTIS_RX28_THREAD_H_
#define _PLUGINS_PANTILT_ROBOTIS_RX28_THREAD_H_

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
  fawkes::PanTiltInterface *pantilt_if_;
  fawkes::LedInterface     *led_if_;
  fawkes::JointInterface   *panjoint_if_;
  fawkes::JointInterface   *tiltjoint_if_;

  fawkes::RefPtr<RobotisRX28> rx28_;

  std::string  pantilt_cfg_prefix_;
  std::string  ptu_cfg_prefix_;
  std::string  ptu_name_;
  std::string  cfg_device_;
  unsigned int cfg_read_timeout_ms_;
  unsigned int cfg_disc_timeout_ms_;
  unsigned int cfg_pan_servo_id_;
  unsigned int cfg_tilt_servo_id_;
  bool         cfg_goto_zero_start_;
  bool         cfg_turn_off_;
  unsigned int cfg_cw_compl_margin_;
  unsigned int cfg_ccw_compl_margin_;
  unsigned int cfg_cw_compl_slope_;
  unsigned int cfg_ccw_compl_slope_;
  float        cfg_pan_min_;
  float        cfg_pan_max_;
  float        cfg_tilt_min_;
  float        cfg_tilt_max_;
  float        cfg_pan_margin_;
  float        cfg_tilt_margin_;
  float        cfg_pan_offset_;
  float        cfg_tilt_offset_;
  float        cfg_pan_start_;
  float        cfg_tilt_start_;
#ifdef HAVE_TF
  std::string  cfg_base_frame_;
  std::string  cfg_pan_link_;
  std::string  cfg_tilt_link_;

  fawkes::tf::Vector3  translation_pan_;
  fawkes::tf::Vector3  translation_tilt_;

  bool         cfg_publish_transforms_;
#endif

  float         last_pan_;
  float         last_tilt_;

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
    fawkes::ReadWriteLock       *rx28_rwlock_;
    fawkes::RefPtr<RobotisRX28>  rx28_;
    fawkes::Logger              *logger_;
    fawkes::WaitCondition       *update_waitcond_;

    unsigned char pan_servo_id_;
    unsigned char tilt_servo_id_;

    float         pan_min_;
    float         pan_max_;
    float         tilt_min_;
    float         tilt_max_;
    float         pan_offset_;
    float         tilt_offset_;
    float         max_pan_speed_;
    float         max_tilt_speed_;
    float         pan_margin_;
    float         tilt_margin_;

    fawkes::ReadWriteLock *value_rwlock_;
    bool  move_pending_;
    float target_pan_;
    float target_tilt_;
    bool  enable_;
    bool  disable_;
    bool  velo_pending_;
    unsigned int pan_vel_;
    unsigned int tilt_vel_;
    bool  led_enable_;
    bool  led_disable_;
    fawkes::Time  pantilt_time_;

    bool fresh_data_;
    fawkes::Mutex *fresh_data_mutex_;

  };

  WorkerThread *wt_;
};

#endif
