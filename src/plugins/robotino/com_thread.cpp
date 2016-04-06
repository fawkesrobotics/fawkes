
/***************************************************************************
 *  com_thread.cpp - Robotino com thread base class
 *
 *  Created: Thu Sep 11 13:18:00 2014
 *  Copyright  2011-2016  Tim Niemueller [www.niemueller.de]
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

#include "com_thread.h"

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

using namespace fawkes;

/** @class RobotinoComThread "com_thread.h"
 * Virtual base class for thread that communicates with a Robotino.
 * A communication thread is always continuous and must communicate at the
 * required pace. It provides hook for sensor and act threads.
 * @author Tim Niemueller
 *
 *
 * @fn void RobotinoComThread::reset_odometry() = 0
 * Reset odometry to zero.
 *
 * @fn bool RobotinoComThread::is_connected() = 0
 * Check if we are connected to OpenRobotino.
 * @return true if the connection has been established, false otherwise
 *
 * @fn void RobotinoComThread::get_act_velocity(float &a1, float &a2, float &a3, unsigned int &seq, fawkes::Time &t) = 0
 * Get actual velocity.
 * @param a1 upon return contains velocity in RPM for first wheel
 * @param a2 upon return contains velocity in RPM for second wheel
 * @param a3 upon return contains velocity in RPM for third wheel
 * @param seq upon return contains sequence number of latest data
 * @param t upon return contains time of latest data
 *
 * @fn bool RobotinoComThread::is_gripper_open() = 0
 * Check if gripper is open.
 * @return true if the gripper is presumably open, false otherwise
 *
 * @fn void RobotinoComThread::set_speed_points(float s1, float s2, float s3) = 0
 * Set speed points for wheels.
 * @param s1 speed point for first wheel in RPM
 * @param s2 speed point for second wheel in RPM
 * @param s3 speed point for third wheel in RPM
 *
 * @fn void RobotinoComThread::set_gripper(bool opened) = 0
 * Open or close gripper.
 * @param opened true to open gripper, false to close
 *
 * @fn void RobotinoComThread::get_odometry(double &x, double &y, double &phi) = 0
 * Get latest odometry value.
 * @param x upon return contains x coordinate of odometry
 * @param y upon return contains y coordinate of odometry
 * @param phi upon return contains rptation of odometry
 *
 * @fn void RobotinoComThread::set_bumper_estop_enabled(bool enabled) = 0
 * Enable or disable emergency stop on bumper contact.
 * @param enabled true to enable, false to disable
 */

/** @class RobotinoComThread::SensorData "com_thread.h"
 * Struct to exchange data between com and sensor thread.
 */

/** Constructor. */
RobotinoComThread::SensorData::SensorData()
	: seq(0), mot_velocity{0,0,0}, mot_position{0,0,0}, mot_current{0.,0.,0.},
	  bumper(false), bumper_estop_enabled(false), digital_in{0,0,0,0,0,0,0,0},
	  analog_in{0.,0.,0.,0.,0.,0.,0.,0.}, bat_voltage(0.), bat_current(0.),
	  imu_enabled(false), imu_orientation{0.,0.,0.,0.}, imu_angular_velocity{0.,0.,0.},
	  imu_angular_velocity_covariance{0.,0.,0.,0.,0.,0.,0.,0.,0.},
	  ir_voltages{0.,0.,0.,0.,0.,0.,0.,0.,0.}
{
}

/** Constructor.
 * @param thread_name name of thread
 */
RobotinoComThread::RobotinoComThread(const char *thread_name)
	: Thread(thread_name, Thread::OPMODE_CONTINUOUS)
{
	data_mutex_  = new Mutex();
	new_data_    = false;
}


/** Destructor. */
RobotinoComThread::~RobotinoComThread()
{
	delete data_mutex_;
}


/** Get all current sensor data.
 * @param sensor_data upon return (true) contains the latest available
 * sensor data
 * @return true if new data was available and has been stored in \p
 * sensor_data, false otherwise
 */
bool
RobotinoComThread::get_data(SensorData &sensor_data)
{
	MutexLocker lock(data_mutex_);
	if (new_data_) {
		sensor_data = data_;
		new_data_ = false;
		return true;
	} else {
		return false;
	}
}
