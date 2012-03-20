
/***************************************************************************
 *  motion_thread.h - Provide NaoQi motion to Fawkes
 *
 *  Created: Thu Jun 09 12:56:42 2011
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

#ifndef __PLUGINS_NAO_MOTION_THREAD_H_
#define __PLUGINS_NAO_MOTION_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/nao/aspect/naoqi.h>

#include <vector>

namespace AL {
  class ALMotionProxy;
  class ALTask;
}
namespace fawkes {
  class HumanoidMotionInterface;
  class NaoSensorInterface;
}

class NaoQiMotionThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::NaoQiAspect
{
 public:
  NaoQiMotionThread();
  virtual ~NaoQiMotionThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void stop_motion();
  void process_messages();
  void fix_angles();

  void goto_body_angles(float head_yaw, float head_pitch,
			float l_shoulder_pitch, float l_shoulder_roll,
			float l_elbow_yaw, float l_elbow_roll,
			float l_wrist_yaw, float l_hand,
			float l_hip_yaw_pitch, float l_hip_roll,
			float l_hip_pitch, float l_knee_pitch,
			float l_ankle_pitch, float l_ankle_roll,
			float r_shoulder_pitch, float r_shoulder_roll,
			float r_elbow_yaw, float r_elbow_roll,
			float r_wrist_yaw, float r_hand,
			float r_hip_yaw_pitch, float r_hip_roll,
			float r_hip_pitch, float r_knee_pitch,
			float r_ankle_pitch, float r_ankle_roll,
			float time_sec);


 private:
  AL::ALPtr<AL::ALMotionProxy> __almotion;
  AL::ALPtr<AL::ALThreadPool>  __thread_pool;

  fawkes::HumanoidMotionInterface *__hummot_if;
  fawkes::NaoSensorInterface      *__sensor_if;

  int __motion_task_id;
  int __head_task_id;
  AL::ALPtr<AL::ALTask>        __motion_task;
};

#endif
