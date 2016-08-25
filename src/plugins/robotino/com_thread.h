/***************************************************************************
 *  com_thread.h - Robotino com thread base class
 *
 *  Created: Thu Sep 11 11:43:42 2014
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

#ifndef __PLUGINS_ROBOTINO_COM_THREAD_H_
#define __PLUGINS_ROBOTINO_COM_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/clock.h>

#include <utils/time/time.h>

#include <cstdio>

namespace fawkes {
	class Mutex;
	class Clock;
}

#define NUM_IR_SENSORS 9

class RobotinoComThread
: public fawkes::Thread,
	public fawkes::ClockAspect,
	public fawkes::LoggingAspect
{
 public:
	struct SensorData {
		SensorData();

		/// @cond INTERNAL
		unsigned int seq;

		float        mot_velocity[3];
		int32_t      mot_position[3];
		float        mot_current[3];
		bool         bumper;
		bool         bumper_estop_enabled;
		bool         digital_in[8];
		bool         digital_out[8];
		float        analog_in[8];

		float        bat_voltage;
		float        bat_current;
		float        bat_absolute_soc;

		bool         imu_enabled;
		float        imu_orientation[4];
		float        imu_angular_velocity[3];
		double       imu_angular_velocity_covariance[9];

		float        odo_x;
		float        odo_y;
		float        odo_phi;

		float        ir_voltages[NUM_IR_SENSORS];
		
		fawkes::Time time;
		/// @endcond
	};

	RobotinoComThread(const char *thread_name);
	virtual ~RobotinoComThread();

	virtual bool is_connected() = 0;

	virtual void set_gripper(bool opened) = 0;
	virtual bool is_gripper_open() = 0;
	virtual void set_speed_points(float s1, float s2, float s3) = 0;
	virtual void get_act_velocity(float &a1, float &a2, float &a3, unsigned int &seq, fawkes::Time &t) = 0;
	virtual void get_odometry(double &x, double &y, double &phi) = 0;

	virtual void reset_odometry() = 0;
	virtual void set_bumper_estop_enabled(bool enabled) = 0;
	virtual void set_motor_accel_limits(float min_accel, float max_accel) = 0;
	virtual void set_digital_output(unsigned int digital_out, bool enable) = 0;
	
	virtual bool get_data(SensorData &sensor_data);

	        void set_drive_layout(float rb, float rw, float gear);
	        void set_drive_limits(float trans_accel, float trans_decel, float rot_accel, float rot_decel);
	virtual void set_desired_vel(float vx, float vy, float omega);

	
	void  project(float *m1, float *m2, float *m3, float vx, float vy, float omega) const;
	void  unproject(float *vx, float *vy, float *omega, float m1, float m2, float m3) const;

 protected:
	bool update_velocities();
	
 private:

	float update_speed(float des, float set, float accel, float decel, float diff_sec);

 protected:
	/** Mutex to protect data_. Lock whenever accessing it. */
	fawkes::Mutex  *data_mutex_;
	/** Data struct that must be updated whenever new data is available. */
	SensorData      data_;
	/** Flag to indicate new data, set to true if data_ is modified. */
	bool            new_data_;

 private:
	float           cfg_rb_;
	float           cfg_rw_;
	float           cfg_gear_;
	float           cfg_trans_accel_;
	float           cfg_trans_decel_;
	float           cfg_rot_accel_;
	float           cfg_rot_decel_;
	
	fawkes::Mutex  *vel_mutex_;
	fawkes::Time   *vel_last_update_;
	bool            vel_last_zero_;
	float           des_vx_;
	float           des_vy_;
	float           des_omega_;

	float           set_vx_;
	float           set_vy_;
	float           set_omega_;

#ifdef USE_VELOCITY_RECORDING
	FILE *f_;
	fawkes::Time     *start_;
#endif
};


#endif

