
/***************************************************************************
 *  sensor_thread.h - Robotino sensor thread
 *
 *  Created: Sun Nov 13 15:33:04 2011
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_ROBOTINO_SENSOR_THREAD_H_
#define __PLUGINS_ROBOTINO_SENSOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <string>
#include <vector>

#define NUM_IR_SENSORS 9

namespace rec {
  namespace robotino {
    namespace com {
      class Com;
    }
  }
  namespace sharedmemory {
    template<typename SharedType> class SharedMemory;
  }
  namespace iocontrol {
    namespace robotstate {
      class State;
    }
  }
}

namespace fawkes {
  class BatteryInterface;
  class RobotinoSensorInterface;
  class IMUInterface;
}

class RobotinoSensorComHandler;

class RobotinoSensorThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ClockAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
  friend class RobotinoActThread;
 public:
  RobotinoSensorThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // methods
  void update_distances(float *distances);

 private: // members
  std::string  cfg_hostname_;
  bool         cfg_quit_on_disconnect_;
  bool         cfg_enable_gyro_;
  std::string  cfg_imu_iface_id_;
  unsigned int cfg_sensor_update_cycle_time_;

  RobotinoSensorComHandler *com_;
  unsigned int last_seqnum_;
  rec::sharedmemory::SharedMemory<rec::iocontrol::robotstate::State> *statemem_;
  rec::iocontrol::robotstate::State *state_;

  fawkes::BatteryInterface        *batt_if_;
  fawkes::RobotinoSensorInterface *sens_if_;
  fawkes::IMUInterface            *imu_if_;

  // Voltage to distance data points
  std::vector<std::pair<double, double> > voltage_to_dist_dps_;
};


#endif
