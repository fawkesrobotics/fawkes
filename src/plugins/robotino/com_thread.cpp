
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
 *
 * @fn void RobotinoComThread::set_motor_accel_limits(float min_accel, float max_accel) = 0
 * Set acceleration limits of motors.
 * @param min_accel minimum acceleration
 * @param max_accel maximum acceleration
 *
 * @fn void RobotinoComThread::set_digital_output(unsigned int digital_out, bool enable) = 0
 * Set digital output state.
 * @param digital_out digital output as written on the robot, i.e., 1 to 8
 * @param enable true to enable output, false to disable
 */

/** @class RobotinoComThread::SensorData "com_thread.h"
 * Struct to exchange data between com and sensor thread.
 */

/** Constructor. */
RobotinoComThread::SensorData::SensorData()
	: seq(0), mot_velocity{0,0,0}, mot_position{0,0,0}, mot_current{0.,0.,0.},
	  bumper(false), bumper_estop_enabled(false), digital_in{0,0,0,0,0,0,0,0},
	  digital_out{0,0,0,0,0,0,0,0},
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

	vel_mutex_ = new Mutex();
	vel_last_update_ = new Time();
	vel_last_zero_ = false;
	des_vx_    = 0.;
	des_vy_    = 0.;
	des_omega_ = 0.;

	set_vx_    = 0.;
	set_vy_    = 0.;
	set_omega_ = 0.;

	cfg_rb_   = 0.;
	cfg_rw_   = 0.;
	cfg_gear_ = 0.;	

	cfg_trans_accel_ = 0.;
	cfg_trans_decel_ = 0.;
	cfg_rot_accel_   = 0.;
	cfg_rot_decel_   = 0.;

#ifdef USE_VELOCITY_RECORDING
	f_ = fopen("comdata.csv", "w");
	start_ = new Time();
#endif
}


/** Destructor. */
RobotinoComThread::~RobotinoComThread()
{
	delete data_mutex_;
#ifdef USE_VELOCITY_RECORDING
	fclose(f_);
#endif
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


/** Set omni drive layout parameters.
 * @param rb Distance from Robotino center to wheel center in meters
 * @param rw Wheel radius in meters
 * @param gear Gear ratio between motors and wheels
 */
void
RobotinoComThread::set_drive_layout(float rb, float rw, float gear)
{
	cfg_rb_   = rb;
	cfg_rw_   = rw;
	cfg_gear_ = gear;	
}


/** Set the omni drive limits.
 * @param trans_accel maximum acceleration in translation
 * @param trans_decel maximum deceleration in translation
 * @param rot_accel maximum acceleration in rotation
 * @param rot_decel maximum deceleration in rotation
 */
void
RobotinoComThread::set_drive_limits(float trans_accel, float trans_decel, float rot_accel, float rot_decel)
{
	cfg_trans_accel_ = trans_accel;
	cfg_trans_decel_ = trans_decel;
	cfg_rot_accel_   = rot_accel;
	cfg_rot_decel_   = rot_decel;
}


/** Set desired velocities.
 * @param vx desired velocity in base_link frame X direction ("forward")
 * @param vy desired velocity in base_link frame Y direction ("sideward")
 * @param omega desired rotational velocity
 */
void
RobotinoComThread::set_desired_vel(float vx, float vy, float omega)
{
	des_vx_    = vx;
	des_vy_    = vy;
	des_omega_ = omega;
}


/** Update velocity values.
 * This method must be called periodically while driving to update the controller.
 * @return true if the method must be called again, false otherwise
 */
bool
RobotinoComThread::update_velocities()
{
	bool set_speed = false;

	Time now(clock);
	float diff_sec = now - vel_last_update_;
	*vel_last_update_ = now;

	set_vx_    = update_speed(des_vx_, set_vx_, cfg_trans_accel_, cfg_trans_decel_, diff_sec);
	set_vy_    = update_speed(des_vy_, set_vy_, cfg_trans_accel_, cfg_trans_decel_, diff_sec);
	set_omega_ = update_speed(des_omega_, set_omega_, cfg_rot_accel_, cfg_rot_decel_, diff_sec);

	/*
	logger->log_info(name(), "VX: %.2f -> %.2f (%.2f)   VY: %.2f -> %.2f (%.2f)   Omg: %.2f -> %.2f (%.2f)",
	                 old_set_vx, set_vx_, des_vx_,
	                 old_set_vy, set_vy_, des_vy_,
	                 old_set_omega, set_omega_);
	*/

	if (set_vx_ == 0.0 && set_vy_ == 0.0 && set_omega_ == 0.0) {
		if (! vel_last_zero_) {
			set_speed = true;
			vel_last_zero_ = true;
		}
	} else {
		set_speed = true;
		vel_last_zero_ = false;
	}

	if (set_speed) {
		float s1 = 0., s2 = 0., s3 = 0.;
		project(&s1, &s2, &s3, set_vx_, set_vy_, set_omega_);
		set_speed_points(s1, s2, s3);

#ifdef USE_VELOCITY_RECORDING
		{
			Time now(clock);
			float time_diff = now - start_;
	
			fprintf(f_, "%f\t%f\t%f\t%f\t%f\t%f\t%f\n", time_diff,
			        des_vx_, set_vx_, des_vy_, set_vy_, des_omega_, set_omega_);
		}
#endif

	}

	return ! vel_last_zero_;
}

float
RobotinoComThread::update_speed(float des, float set, float accel, float decel, float diff_sec)
{
	if (des >= 0 && set < 0) {
		const float decrement = std::copysign(decel, set) * diff_sec;
		if (des > set - decrement) {
			//logger->log_debug(name(), "    Case 1a  %f  %f  %f", decrement, decel, diff_sec);
			set -= decrement;
		} else {
			//logger->log_debug(name(), "    Case 1b");
			set = des;
		}

	} else if (des <= 0 && set > 0) {
		const float decrement = std::copysign(decel, set) * diff_sec;
		if (des < set - decrement ) {
			//logger->log_debug(name(), "    Case 1c  %f  %f  %f", decrement, decel, diff_sec);
			set -= decrement;
		} else {
			//logger->log_debug(name(), "    Case 1d");
			set = des;
		}

	} else if (fabs(des) > fabs(set)) {
		const float increment = std::copysign(accel, des) * diff_sec;
		if (fabs(des) > fabs(set + increment)) {
			//logger->log_debug(name(), "    Case 2a  %f  %f", increment, accel, diff_sec);
			set += increment;
		} else {
			//logger->log_debug(name(), "    Case 2b");
			set  = des;
		}
	} else if (fabs(des) < fabs(set)) {
		const float decrement = std::copysign(decel, des) * diff_sec;
		if (fabs(des) < fabs(set - decrement)) {
			//logger->log_debug(name(), "    Case 3a  %f  %f  %f", decrement, decel, diff_sec);
			set -= decrement;
		} else {
			//logger->log_debug(name(), "    Case 3b");
			set  = des;
		}
	}

	return set;
}

/** Project the velocity of the robot in cartesian coordinates to single motor speeds.
 *
 * From OpenRobotino API2 (C) REC Robotics Equipment Corporation GmbH, Planegg, Germany.
 * The code has been released under a 2-clause BSD license.
 *
 * @param m1		The resulting speed of motor 1 in rpm
 * @param m2		The resulting speed of motor 2 in rpm
 * @param m3		The resulting speed of motor 3 in rpm
 * @param vx		Velocity in x-direction in m/s
 * @param vy		Velocity in y-direction in m/s
 * @param omega	Angular velocity in rad/s
 */
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//1) Redistributions of source code must retain the above copyright
//notice, this list of conditions and the following disclaimer.
//2) Redistributions in binary form must reproduce the above copyright
//notice, this list of conditions and the following disclaimer in the
//documentation and/or other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY REC ROBOTICS EQUIPMENT CORPORATION GMBH
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REC
//ROBOTICS EQUIPMENT CORPORATION GMBH BE LIABLE FOR ANY DIRECT,
//INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
//STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
//OF THE POSSIBILITY OF SUCH DAMAGE.
void
RobotinoComThread::project( float* m1, float* m2, float* m3, float vx, float vy, float omega ) const
{
	//Projection matrix
	static const double v0[2] = { -0.5 * sqrt( 3.0 ),  0.5 };
	static const double v1[2] = {  0.0              , -1.0 };
	static const double v2[2] = {  0.5 * sqrt( 3.0 ),  0.5 };

	//Scale omega with the radius of the robot
	double vOmegaScaled = cfg_rb_ * (double)omega ;

	//Convert from m/s to RPM
	const double k = 60.0 * cfg_gear_ / ( 2.0 * M_PI * cfg_rw_ );

	//Compute the desired velocity
	*m1 = static_cast<float>( ( v0[0] * (double)vx + v0[1] * (double)vy + vOmegaScaled ) * k );
	*m2 = static_cast<float>( ( v1[0] * (double)vx + v1[1] * (double)vy + vOmegaScaled ) * k );
	*m3 = static_cast<float>( ( v2[0] * (double)vx + v2[1] * (double)vy + vOmegaScaled ) * k );
}

/** Project single motor speeds to velocity in cartesian coordinates.
 *
 * From OpenRobotino API2 (C) REC Robotics Equipment Corporation GmbH, Planegg, Germany.
 * The code has been released under a 2-clause BSD license.
 *
 * @param vx		The resulting speed in x-direction in m/s
 * @param vy		The resulting speed in y-direction in m/s
 * @param omega	The resulting angular velocity in rad/s
 * @param m1		Speed of motor 1 in rpm
 * @param m2		Speed of motor 2 in rpm
 * @param m3		Speed of motor 3 in rpm
 * @throws		RobotinoException if no valid drive layout parameters are available.
 */
void
RobotinoComThread::unproject( float* vx, float* vy, float* omega, float m1, float m2, float m3 ) const
{
	//Convert from RPM to mm/s
	const double k = 60.0 * cfg_gear_ / ( 2.0 * M_PI * cfg_rw_ );

	*vx = static_cast<float>( ( (double)m3 - (double)m1 ) / sqrt( 3.0 ) / k );
	*vy = static_cast<float>( 2.0 / 3.0 * ( (double)m1 + 0.5 * ( (double)m3 - (double)m1 ) - (double)m2 ) / k );

	double vw = (double)*vy + (double)m2 / k;

	*omega = static_cast<float>( vw / cfg_rb_ );
}
