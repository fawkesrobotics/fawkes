
/***************************************************************************
 *  act_thread.h - Katana plugin act thread
 *
 *  Created: Mon Jun 08 17:59:57 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *             2010-2014  Bahram Maleki-Fard
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

#ifndef __PLUGINS_KATANA_ACT_THREAD_H_
#define __PLUGINS_KATANA_ACT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/tf.h>
#ifdef HAVE_OPENRAVE
#  include <plugins/openrave/aspect/openrave.h>
#endif
#include <blackboard/interface_listener.h>
#include <core/utils/refptr.h>
#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <string>
#include <memory>
#include <vector>

namespace fawkes {
  class KatanaInterface;
  class JointInterface;
  class Time;
  class KatanaController;
}

class KatanaSensorAcquisitionThread;
class KatanaMotionThread;
class KatanaCalibrationThread;
class KatanaGotoThread;
class KatanaGripperThread;
class KatanaMotorControlThread;
class KatanaGotoOpenRaveThread;

class KatanaActThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::TransformAspect,
#ifdef HAVE_OPENRAVE
  public fawkes::OpenRaveAspect,
#endif
  public fawkes::BlackBoardInterfaceListener
{
 public:
  KatanaActThread();
  ~KatanaActThread();

  virtual void init();
  virtual void finalize();
  virtual void once();
  virtual void loop();

  // For BlackBoardInterfaceListener
  virtual bool bb_interface_message_received(fawkes::Interface *interface,
               fawkes::Message *message) throw();

  void update_sensor_values();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void stop_motion();
  void update_position(bool refresh);
  void update_sensors(bool refresh);
  void update_motors(bool refresh);
  void start_motion(fawkes::RefPtr<KatanaMotionThread> motion_thread,
                    unsigned int msgid, const char *logmsg, ...);

 private:
  fawkes::KatanaInterface *__katana_if;
  std::vector<fawkes::JointInterface*> *__joint_ifs;

  std::string    __cfg_controller;
  std::string    __cfg_device;
  std::string    __cfg_kni_conffile;
  bool           __cfg_auto_calibrate;
  unsigned int   __cfg_defmax_speed;
  unsigned int   __cfg_read_timeout;
  unsigned int   __cfg_write_timeout;
  unsigned int   __cfg_gripper_pollint;
  unsigned int   __cfg_goto_pollint;
  float          __cfg_park_x;
  float          __cfg_park_y;
  float          __cfg_park_z;
  float          __cfg_park_phi;
  float          __cfg_park_theta;
  float          __cfg_park_psi;

  float          __cfg_distance_scale;

  float          __cfg_update_interval;

  std::string    __cfg_frame_kni;
  std::string    __cfg_frame_gripper;
  std::string    __cfg_frame_openrave;

  bool           __cfg_OR_enabled;
#ifdef HAVE_OPENRAVE
  bool           __cfg_OR_use_viewer;
  bool           __cfg_OR_auto_load_ik;
  std::string    __cfg_OR_robot_file;
  std::string    __cfg_OR_arm_model;
#endif

  fawkes::RefPtr<KatanaSensorAcquisitionThread> __sensacq_thread;
  fawkes::RefPtr<KatanaMotionThread>           __actmot_thread;
  fawkes::RefPtr<KatanaCalibrationThread>      __calib_thread;
  fawkes::RefPtr<KatanaGotoThread>             __goto_thread;
  fawkes::RefPtr<KatanaGripperThread>          __gripper_thread;
  fawkes::RefPtr<KatanaMotorControlThread>     __motor_control_thread;
#ifdef HAVE_OPENRAVE
  fawkes::RefPtr<KatanaGotoOpenRaveThread>     __goto_openrave_thread;
#endif

  fawkes::RefPtr<fawkes::KatanaController>      __katana;

  fawkes::Time                  *__last_update;

#ifdef USE_TIMETRACKER
  fawkes::RefPtr<fawkes::TimeTracker> __tt;
  unsigned int __tt_count;
  unsigned int __ttc_read_sensor;
#endif
};


#endif
