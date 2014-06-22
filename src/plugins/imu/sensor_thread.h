
/***************************************************************************
 *  sensor_thread.h - IMU thread that pushes data into the interface
 *
 *  Created: Sun Jun 22 19:34:03 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_IMU_SENSOR_THREAD_H_
#define __PLUGINS_IMU_SENSOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <string>

namespace fawkes {
  class IMUInterface;
}

class IMUAcquisitionThread;

class IMUSensorThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  IMUSensorThread(std::string &cfg_name, std::string &cfg_prefix,
		    IMUAcquisitionThread *aqt);

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::IMUInterface *imu_if_;

  IMUAcquisitionThread *aqt_;

  std::string             cfg_name_;
  std::string             cfg_frame_;
  std::string             cfg_prefix_;
};


#endif
