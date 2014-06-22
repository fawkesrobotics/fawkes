
/***************************************************************************
 *  sensor_thread.cpp - IMU thread that pushes data into the interface
 *
 *  Created: Sun Jun 22 19:42:14 2014
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

#include "sensor_thread.h"
#include "acquisition_thread.h"

#include <interfaces/IMUInterface.h>

using namespace fawkes;

/** @class IMUSensorThread "sensor_thread.h"
 * IMU sensor thread.
 * This thread integrates into the Fawkes main loop at the sensor hook and
 * publishes new data when available from the IMUAcquisitionThread.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param cfg_name short name of configuration group
 * @param cfg_prefix configuration path prefix
 * @param aqt IMUAcquisitionThread to get data from
 */
IMUSensorThread::IMUSensorThread(std::string &cfg_name,
				     std::string &cfg_prefix,
				     IMUAcquisitionThread *aqt)
  : Thread("IMUSensorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
  set_name("IMUSensorThread(%s)", cfg_name.c_str());
  aqt_        = aqt;
  cfg_name_   = cfg_name;
  cfg_prefix_ = cfg_prefix;
}


void
IMUSensorThread::init()
{
  imu_if_ = NULL;

  cfg_frame_ = config->get_string((cfg_prefix_ + "frame").c_str());

  std::string if_id = "IMU " + cfg_name_;

  imu_if_ = blackboard->open_for_writing<IMUInterface>(if_id.c_str());
  imu_if_->set_auto_timestamping(false);
  imu_if_->set_frame(cfg_frame_.c_str());
  imu_if_->write();
}


void
IMUSensorThread::finalize()
{
  blackboard->close(imu_if_);
}

void
IMUSensorThread::loop()
{
  if (aqt_->lock_if_new_data()) {
    imu_if_->set_timestamp(aqt_->get_timestamp());
    imu_if_->set_orientation(aqt_->get_orientation());
    imu_if_->set_orientation_covariance(aqt_->get_orientation_covariance());
    imu_if_->set_angular_velocity(aqt_->get_angular_velocity());
    imu_if_->set_angular_velocity_covariance(aqt_->get_angular_velocity_covariance());
    imu_if_->set_linear_acceleration(aqt_->get_linear_acceleration());
    imu_if_->set_linear_acceleration_covariance(aqt_->get_linear_acceleration_covariance());
    imu_if_->write();
    aqt_->unlock();
  }
}
