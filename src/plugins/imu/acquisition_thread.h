
/***************************************************************************
 *  acquisition_thread.h - Thread that retrieves IMU data
 *
 *  Created: Sun Jun 22 21:16:03 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_IMU_ACQUISITION_THREAD_H_
#define __PLUGINS_IMU_ACQUISITION_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>

namespace fawkes {
  class Mutex;
  class Configuration;
  class Logger;
  class Time;
  class IMUInterface;
}

class IMUAcquisitionThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect
{
 public:
  IMUAcquisitionThread(const char *thread_name, bool continuous,
		       std::string &cfg_name, std::string &cfg_prefix);
  virtual ~IMUAcquisitionThread();

  bool lock_if_new_data();
  void unlock();

  // must be called from sub-classes in continuous case
  virtual void init();
  virtual void loop();
  virtual void finalize();

  /** Get orientation data.
   * @return orientation data */
  const float *  get_orientation()
  { return orientation_; }

  /** Get orientation covariance.
   * @return orientation covariance */
  const double *  get_orientation_covariance()
  { return orientation_covariance_; }

  /** Get angular velocity data.
   * @return angular velocity data */
  const float *  get_angular_velocity()
  { return angular_velocity_; }

  /** Get angular velocity covariance
   * @return angular velocity covariance */
  const double *  get_angular_velocity_covariance()
  { return angular_velocity_covariance_; }

  /** Get linear acceleration data.
   * @return linear acceleration data */
  const float *  get_linear_acceleration()
  { return linear_acceleration_; }

  /** Get linera acceleration covariance.
   * @return linear acceleration covariance */
  const double *  get_linear_acceleration_covariance()
  { return linear_acceleration_covariance_; }

  /** Get time of data set.
   * @return timestamp */
  const fawkes::Time *   get_timestamp()
  { return timestamp_; }

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 protected:
  std::string  cfg_name_;
  std::string  cfg_prefix_;
  std::string  cfg_frame_;
  bool         cfg_continuous_;

  fawkes::Mutex    *data_mutex_;
  fawkes::Time     *timestamp_;

  bool   new_data_;

  float  orientation_[4];
  double orientation_covariance_[9];
  float  angular_velocity_[3];
  double angular_velocity_covariance_[9];
  float  linear_acceleration_[3];
  double linear_acceleration_covariance_[9];

 private:
  // only used if continuous
  fawkes::IMUInterface *imu_if_;

};


#endif
