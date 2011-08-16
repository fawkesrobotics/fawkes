
/***************************************************************************
 *  dcm_thread.h - Provide NaoQi DCM to Fawkes
 *
 *  Created: Tue May 31 14:59:30 2011
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

#ifndef __PLUGINS_NAO_DCM_THREAD_H_
#define __PLUGINS_NAO_DCM_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/nao/aspect/naoqi.h>

#include <core/utils/lock_vector.h>

#include <interfaces/NaoJointPositionInterface.h>
#include <interfaces/NaoSensorInterface.h>

#include <althread/alprocesssignals.h>
#include <alcommon/alproxy.h>

#include <vector>

namespace AL {
  class ALMemoryFastAccess;
}
namespace fawkes {
  class NaoJointStiffnessInterface;
}

class NaoQiDCMThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::NaoQiAspect
{
 public:
  NaoQiDCMThread();
  virtual ~NaoQiDCMThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void dcm_callback();
  void read_values();
  void update_interfaces(fawkes::NaoJointPositionInterface *joint_pos_if,
			 fawkes::NaoJointStiffnessInterface *joint_stiffness_if,
			 fawkes::NaoSensorInterface *sensor_if);
  void process_messages();
  std::vector<std::string> parse_servo_bitfield(unsigned int servos);


  void send_commands(unsigned int servos, std::string what,
		     float value, int time_offset);
  void send_command(std::string name, float value,
                    std::string kind, int time_offset);

  int ultrasonic_value(fawkes::NaoSensorInterface::UltrasonicDirection direction);

  class HighFreqThread;
  HighFreqThread *__highfreq_thread;

 private:
  AL::ALPtr<AL::DCMProxy> __dcm;
  AL::ALPtr<AL::ALMotionProxy> __almotion;
  AL::ALPtr<AL::ALMemoryFastAccess> __memfa;
  bool __robocup_version;

  AL::ALProcessSignals::ProcessSignalConnection __dcm_sigconn;

  int                       __dcm_time;
  fawkes::LockVector<float> __values;

  fawkes::NaoJointPositionInterface   *__joint_pos_highfreq_if;
  fawkes::NaoJointPositionInterface   *__joint_pos_if;
  fawkes::NaoJointStiffnessInterface  *__joint_stiffness_highfreq_if;
  fawkes::NaoJointStiffnessInterface  *__joint_stiffness_if;
  fawkes::NaoSensorInterface          *__sensor_highfreq_if;
  fawkes::NaoSensorInterface          *__sensor_if;

  uint8_t                                      __robot_version[4];
  fawkes::NaoJointPositionInterface::RobotType __robot_type;
  int                                          __usboard_version;

  AL::ALValue __alljoint_names;
};

#endif
