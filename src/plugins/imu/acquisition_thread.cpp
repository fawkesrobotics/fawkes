
/***************************************************************************
 *  acqusition_thread.cpp - Thread that retrieves IMU data
 *
 *  Created: Sun Jun 22 21:18:41 2014
 *  Copyright  2008-2014  Tim Niemueller [www.niemueller.de]
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

#include "acquisition_thread.h"

#include <core/threading/mutex.h>
#include <interfaces/IMUInterface.h>

#include <limits>
#include <cstring>
#include <cstdlib>

using namespace fawkes;

/** @class IMUAcquisitionThread "acquisition_thread.h"
 * IMU acqusition thread.
 * Interface for different laser types.
 * @author Tim Niemueller
 */

/** @var fawkes::Mutex * IMUAcquisitionThread::data_mutex_
 * Lock while writing to distances or echoes array or marking new data
 */

/** @var bool IMUAcquisitionThread::new_data_
 * Set to true in your loop if new data is available. Set to false automatically
 * in get_distance_data() and get_echoes_data().
 */

/** @var float * IMUAcquisitionThread::orientation_
 * Pre-allocated orientation quaternion as array, 4 entries ordered (x,y,z,w).
 */

/** @var float * IMUAcquisitionThread::orientation_covariance_
 * Pre-allocated orientation covariance, row major matrix ordered x, y, z.
 */

/** @var float * IMUAcquisitionThread::angular_velocity_
 * Pre-allocated angular velocities as array, 3 entries ordered (x,y,z).
 */

/** @var float * IMUAcquisitionThread::angular_velocity_covariance_
 * Pre-allocated angular velocity covariance, row major matrix ordered x, y, z.
 */

/** @var float * IMUAcquisitionThread::linear_acceleration_
 * Pre-allocated linear acceleration as array, 3 entries ordered (x,y,z).
 */

/** @var float * IMUAcquisitionThread::linear_acceleration_covariance_
 * Pre-allocated linear acceleration covariance, row major matrix ordered x, y, z.
 */

/** @var fawkes::Time * IMUAcquisitionThread::timestamp_
 * Time when the most recent data was received.
 */

/** @var std::string IMUAcquisitionThread::cfg_name_
 * Configuration name (third element in config path).
 */

/** @var std::string IMUAcquisitionThread::cfg_prefix_
 * Configuration path prefix.
 */

/** @var std::string IMUAcquisitionThread::cfg_frame_
 * Coordinate frame for sensor.
 */

/** @var std::string IMUAcquisitionThread::cfg_continuous_
 * True if running continuous.
 * Sub-classes must call init(), loop(), and finalize().
 */


/** Constructor.
 * @param thread_name name of the thread, be descriptive
 * @param continuous true to run continuous, false otherwise
 * @param cfg_name configuration name
 * @param cfg_prefix configuration path prefix
 */
IMUAcquisitionThread::IMUAcquisitionThread(const char *thread_name, bool continuous,
					   std::string &cfg_name, std::string &cfg_prefix)
  : Thread(thread_name, Thread::OPMODE_CONTINUOUS)
{
  cfg_name_       = cfg_name;
  cfg_prefix_     = cfg_prefix;
  cfg_continuous_ = continuous;

  data_mutex_ = new Mutex();
  timestamp_  = new Time();
  new_data_   = false;

  for (unsigned int i = 0; i < 4; ++i)  orientation_[i] = 0.;
  for (unsigned int i = 0; i < 9; ++i)  orientation_covariance_[i] = 0.;
  for (unsigned int i = 0; i < 3; ++i)  angular_velocity_[i] = 0.;
  for (unsigned int i = 0; i < 9; ++i)  angular_velocity_covariance_[i] = 0.;
  for (unsigned int i = 0; i < 3; ++i)  linear_acceleration_[i] = 0.;
  for (unsigned int i = 0; i < 9; ++i)  linear_acceleration_covariance_[i] = 0.;

}

IMUAcquisitionThread::~IMUAcquisitionThread()
{
  delete data_mutex_;
  delete timestamp_;
}


void
IMUAcquisitionThread::init()
{
  if (! cfg_continuous_)  return;

  imu_if_ = NULL;
  cfg_frame_ = config->get_string((cfg_prefix_ + "frame").c_str());

  std::string if_id = "IMU " + cfg_name_;

  imu_if_ = blackboard->open_for_writing<IMUInterface>(if_id.c_str());
  imu_if_->set_auto_timestamping(false);
  imu_if_->set_frame(cfg_frame_.c_str());
  imu_if_->write();
}


void
IMUAcquisitionThread::finalize()
{
  blackboard->close(imu_if_);
}


void
IMUAcquisitionThread::loop()
{
  data_mutex_->lock();
  if (new_data_) {
    imu_if_->set_timestamp(timestamp_);
    imu_if_->set_orientation(orientation_);
    imu_if_->set_orientation_covariance(orientation_covariance_);
    imu_if_->set_angular_velocity(angular_velocity_);
    imu_if_->set_angular_velocity_covariance(angular_velocity_covariance_);
    imu_if_->set_linear_acceleration(linear_acceleration_);
    imu_if_->set_linear_acceleration_covariance(linear_acceleration_covariance_);
    imu_if_->write();
  }
  data_mutex_->unlock();
}

/** Lock data if fresh.
 * If new data has been received since get_distance_data() or get_echo_data()
 * was called last the data is locked, no new data can arrive until you call
 * unlock(), otherwise the lock is immediately released after checking.
 * @return true if the lock was acquired and there is new data, false otherwise
 */
bool
IMUAcquisitionThread::lock_if_new_data()
{
  data_mutex_->lock();
  if (new_data_) {
    return true;
  } else {
    data_mutex_->unlock();
    return false;
  }
}


/** Unlock data, */
void
IMUAcquisitionThread::unlock()
{
  data_mutex_->unlock();
}
