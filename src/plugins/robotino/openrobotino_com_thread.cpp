
/***************************************************************************
 *  openrobotino_com_thread.cpp - Robotino com thread
 *
 *  Created: Thu Sep 11 13:18:00 2014
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
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

#include "openrobotino_com_thread.h"
#ifdef HAVE_OPENROBOTINO_API_1
#  include <rec/robotino/com/Com.h>
#  include <rec/sharedmemory/sharedmemory.h>
#  include <rec/iocontrol/remotestate/SensorState.h>
#  include <rec/iocontrol/robotstate/State.h>
#else
#  include <rec/robotino/api2/Com.h>
#  include <rec/robotino/api2/AnalogInputArray.h>
#  include <rec/robotino/api2/Bumper.h>
#  include <rec/robotino/api2/DigitalInputArray.h>
#  include <rec/robotino/api2/DistanceSensorArray.h>
#  include <rec/robotino/api2/ElectricalGripper.h>
#  include <rec/robotino/api2/Gyroscope.h>
#  include <rec/robotino/api2/MotorArray.h>
#  include <rec/robotino/api2/Odometry.h>
#  include <rec/robotino/api2/PowerManagement.h>
#endif
#include <baseapp/run.h>
#include <utils/math/angle.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <utils/time/wait.h>
#include <tf/types.h>

#include <unistd.h>

using namespace fawkes;


/** @class OpenRobotinoComThread "openrobotino_com_thread.h"
 * Thread to communicate with Robotino via OpenRobotino API (v1 or v2).
 * @author Tim Niemueller
 */

/** Constructor. */
OpenRobotinoComThread::OpenRobotinoComThread()
	: RobotinoComThread("OpenRobotinoComThread")
{
#ifdef HAVE_OPENROBOTINO_API_1
	com_ = this;
#endif
}


/** Destructor. */
OpenRobotinoComThread::~OpenRobotinoComThread()
{
}


void
OpenRobotinoComThread::init()
{
	cfg_hostname_ = config->get_string("/hardware/robotino/openrobotino/hostname");
	cfg_quit_on_disconnect_ = config->get_bool("/hardware/robotino/openrobotino/quit_on_disconnect");
	cfg_sensor_update_cycle_time_ =
		config->get_uint("/hardware/robotino/cycle-time");
	cfg_gripper_enabled_ = config->get_bool("/hardware/robotino/gripper/enable_gripper");
	cfg_enable_gyro_ = config->get_bool("/hardware/robotino/gyro/enable");

#ifdef HAVE_OPENROBOTINO_API_1
	statemem_ =  new rec::sharedmemory::SharedMemory<rec::iocontrol::robotstate::State>
		(rec::iocontrol::robotstate::State::sharedMemoryKey);
	state_ = statemem_->getData();
	state_mutex_ = new Mutex();
	set_state_ = new rec::iocontrol::remotestate::SetState();
	set_state_->gripper_isEnabled = cfg_gripper_enabled_;
	active_state_ = 0;
#endif

	last_seqnum_ = 0;
	time_wait_   = new TimeWait(clock, cfg_sensor_update_cycle_time_ * 1000);

	if (cfg_enable_gyro_) {
		data_.imu_enabled = true;

		for (int i = 0; i < 3; ++i)  data_.imu_angular_velocity[i] = 0.;
		for (int i = 0; i < 4; ++i)  data_.imu_orientation[i] = 0.;
		for (int i = 0; i < 9; ++i)  data_.imu_angular_velocity_covariance[i] = 0.;
		// Assume that the gyro is the CruizCore XG1010 and thus set data
		// from datasheet
		data_.imu_angular_velocity_covariance[8] = deg2rad(0.1);
	}
	
#ifdef HAVE_OPENROBOTINO_API_1
	com_->setAddress(cfg_hostname_.c_str());
	com_->setMinimumUpdateCycleTime(cfg_sensor_update_cycle_time_);
	com_->connect(/* blocking */ false);
#else
	com_ = new rec::robotino::api2::Com("Fawkes");
	com_->setAddress(cfg_hostname_.c_str());
	com_->setAutoReconnectEnabled(false);
	com_->connectToServer(/* blocking */ true);

	analog_inputs_com_  = new rec::robotino::api2::AnalogInputArray();
	bumper_com_         = new rec::robotino::api2::Bumper();
	digital_inputs_com_ = new rec::robotino::api2::DigitalInputArray();
	distances_com_      = new rec::robotino::api2::DistanceSensorArray();
	gripper_com_        = new rec::robotino::api2::ElectricalGripper();
	gyroscope_com_      = new rec::robotino::api2::Gyroscope();
	motors_com_         = new rec::robotino::api2::MotorArray();
	odom_com_           = new rec::robotino::api2::Odometry();
	power_com_          = new rec::robotino::api2::PowerManagement();

	analog_inputs_com_->setComId(com_->id());
	bumper_com_->setComId(com_->id());
	digital_inputs_com_->setComId(com_->id());
	distances_com_->setComId(com_->id());
	gripper_com_->setComId(com_->id());
	gyroscope_com_->setComId(com_->id());
	motors_com_->setComId(com_->id());
	odom_com_->setComId(com_->id());
	power_com_->setComId(com_->id());
#endif

}


void
OpenRobotinoComThread::finalize()
{
	delete time_wait_;
#ifdef HAVE_OPENROBOTINO_API_1
	set_state_->speedSetPoint[0] = 0.;
	set_state_->speedSetPoint[1] = 0.;
	set_state_->speedSetPoint[2] = 0.;
	set_state_->gripper_isEnabled = false;
	com_->setSetState(*set_state_);
	usleep(50000);
	delete set_state_;
	delete state_mutex_;
	delete statemem_;
#else
	float speeds[3] = { 0, 0, 0 };
	motors_com_->setSpeedSetPoints(speeds, 3);
	usleep(50000);
	delete analog_inputs_com_;
	delete bumper_com_;
	delete digital_inputs_com_;
	delete distances_com_;
	delete gripper_com_;
	delete gyroscope_com_;
	delete motors_com_;
	delete odom_com_;
	delete power_com_;
	delete com_;
#endif
}

void
OpenRobotinoComThread::once()
{
	reset_odometry();
}

void
OpenRobotinoComThread::loop()
{
	time_wait_->mark_start();

	if (com_->isConnected()) {
		MutexLocker lock(data_mutex_);
#ifdef HAVE_OPENROBOTINO_API_1
		process_api_v1();
#else
		process_api_v2();
#endif


#ifdef HAVE_OPENROBOTINO_API_1
	} else if (com_->connectionState() == rec::robotino::com::Com::NotConnected) {
#else
	} else {
#endif
		if (cfg_quit_on_disconnect_) {
			logger->log_warn(name(), "Connection lost, quitting (as per config)");
			fawkes::runtime::quit();
		} else {
			// retry connection
#ifdef HAVE_OPENROBOTINO_API_1
			com_->connect(/* blocking */ false);
#else
			com_->connectToServer(/* blocking */ true);
#endif
		}
	}

	time_wait_->wait();
}


void
OpenRobotinoComThread::process_api_v1()
{
#ifdef HAVE_OPENROBOTINO_API_1
	state_mutex_->lock();
	fawkes::Time sensor_time = times_[active_state_];
	rec::iocontrol::remotestate::SensorState sensor_state = sensor_states_[active_state_];
	state_mutex_->unlock();

	if (sensor_state.sequenceNumber != last_seqnum_) {
		new_data_ = true;
		last_seqnum_ = sensor_state.sequenceNumber;

		// update sensor values in interface
		for (int i = 0; i < 3; ++i) {
			data_.mot_velocity[i] = sensor_state.actualVelocity[i];
			data_.mot_position[i] = sensor_state.actualPosition[i];
			data_.mot_current[i] = sensor_state.motorCurrent[i];
		}
		data_.bumper = sensor_state.bumper;
		data_.bumper_estop_enabled = state_->emergencyStop.isEnabled;
		for (int i = 0; i < 8; ++i) {
			data_.digital_in[i] = sensor_state.dIn[i];
			data_.analog_in[i]  = sensor_state.aIn[i];
		}
		if (cfg_enable_gyro_) {
			if (state_->gyro.port == rec::serialport::UNDEFINED) {
				if (fabs(data_.imu_angular_velocity[0] + 1.) > 0.00001) {
					data_.imu_angular_velocity[0] = -1.;
					data_.imu_angular_velocity[2] =  0.;
					data_.imu_orientation[0] = -1.;
				}
			} else {
				data_.imu_angular_velocity[0] = 0.;
				data_.imu_angular_velocity[2] = state_->gyro.rate;

				tf::Quaternion q = tf::create_quaternion_from_yaw(state_->gyro.angle);
				data_.imu_orientation[0] = q.x();
				data_.imu_orientation[1] = q.y();
				data_.imu_orientation[2] = q.z();
				data_.imu_orientation[3] = q.w();
			}
		}

		for (int i = 0; i < NUM_IR_SENSORS; ++i) {
			data_.ir_voltages[i] = sensor_state.distanceSensor[i];
		}

		data_.bat_voltage = roundf(sensor_state.voltage * 1000.);
		data_.bat_current = roundf(sensor_state.current);

		// 21.0V is empty, 26.0V is empty, from OpenRobotino lcdd
		float soc = (sensor_state.voltage - 21.0f) / 5.f;
		soc = std::min(1.f, std::max(0.f, soc));
		data_.bat_absolute_soc = soc;
	}
#endif
}


void
OpenRobotinoComThread::process_api_v2()
{
#ifdef HAVE_OPENROBOTINO_API_2
	com_->processComEvents();

	double odo_x = 0, odo_y = 0, odo_phi = 0;
	unsigned int odo_seq = 0;

	odom_com_->readings(&odo_x, &odo_y, &odo_phi, &odo_seq);

	if (odo_seq != last_seqnum_) {  
		new_data_ = true;
		last_seqnum_ = odo_seq;

		if (motors_com_->numMotors() != 3) {
			logger->log_error(name(), "Invalid number of motors, got %u, expected 3",
			                  motors_com_->numMotors());
			return;
		}
		motors_com_->actualVelocities(data_.mot_velocity);
		motors_com_->actualPositions(data_.mot_position);
		motors_com_->motorCurrents(data_.mot_current);

		data_.bumper = bumper_com_->value();

		if (digital_inputs_com_->numDigitalInputs() != 8) {
			logger->log_error(name(), "Invalid number of digital inputs, got %u, expected 8",
			                  digital_inputs_com_->numDigitalInputs());
			return;
		}
		int digin_readings[8];
		digital_inputs_com_->values(digin_readings);
		for (unsigned int i = 0; i < 8; ++i)  data_.digital_in[i] = (digin_readings[i] != 0);

		if (analog_inputs_com_->numAnalogInputs() != 8) {
			logger->log_error(name(), "Invalid number of analog inputs, got %u, expected 8",
			                  analog_inputs_com_->numAnalogInputs());
			return;
		}
		analog_inputs_com_->values(data_.analog_in);

		if (distances_com_->numDistanceSensors() != NUM_IR_SENSORS) {
			logger->log_error(name(), "Invalid number of distance sensors, got %u, expected 9",
			                  distances_com_->numDistanceSensors());
			return;
		}
		// the distance calculation from API2 uses a max value of 0.41,
		// which breaks the previous behavior of 0.0 for "nothing"
		// therefore use our API1 conversion routine
		distances_com_->voltages(data_.ir_voltages);

		float pow_current = power_com_->current() * 1000.; // A -> mA
		float pow_voltage = power_com_->voltage() * 1000.; // V -> mV

		float gyro_angle  = gyroscope_com_->angle();
		float gyro_rate   = gyroscope_com_->rate();

		data_.bumper_estop_enabled = false;
		data_.imu_angular_velocity[0] = 0.;
		data_.imu_angular_velocity[2] = gyro_rate;

		tf::Quaternion q = tf::create_quaternion_from_yaw(gyro_angle);
		data_.imu_orientation[0] = q.x();
		data_.imu_orientation[1] = q.y();
		data_.imu_orientation[2] = q.z();
		data_.imu_orientation[3] = q.w();

		data_.bat_voltage = roundf(pow_voltage);;
		data_.bat_current = roundf(pow_current);

		// 22.0V is empty, 24.5V is full, this is just a guess
		float soc = (power_com_->voltage() - 22.0f) / 2.5f;
		soc = std::min(1.f, std::max(0.f, soc));
		data_.bat_absolute_soc = soc;
	}
#endif
}

#ifdef HAVE_OPENROBOTINO_API_1
/** Update event. */
void
OpenRobotinoComThread::updateEvent()
{
	unsigned int next_state = 1 - active_state_;
	sensor_states_[next_state] = sensorState();
	times_[next_state].stamp();

	MutexLocker lock(state_mutex_);
	active_state_ = next_state;
}
#endif


/** Reset odometry to zero. */
void
OpenRobotinoComThread::reset_odometry()
{
	if (com_->isConnected()) {
#ifdef HAVE_OPENROBOTINO_API_1
		set_state_->setOdometry = true;
		set_state_->odometryX = set_state_->odometryY = set_state_->odometryPhi = 0.;
		com_->setSetState(*set_state_);
		set_state_->setOdometry = false;
#else
		odom_com_->set(0., 0., 0., /* blocking */ true);
#endif
	}
}


void
OpenRobotinoComThread::set_motor_accel_limits(float min_accel, float max_accel)
{
	throw Exception("Setting motor accel limits for OpenRobotino driver not supported, configure controld3");
}

void
OpenRobotinoComThread::set_digital_output(unsigned int digital_out, bool enable)
{
	logger->log_error(name(), "Setting digital outputs not supported with openrobotino driver");
}


/** Check if we are connected to OpenRobotino.
 * @return true if the connection has been established, false otherwise
 */
bool
OpenRobotinoComThread::is_connected()
{
	return com_->isConnected();
}


/** Get actual velocity.
 * @param a1 upon return contains velocity in RPM for first wheel
 * @param a2 upon return contains velocity in RPM for second wheel
 * @param a3 upon return contains velocity in RPM for third wheel
 * @param seq upon return contains sequence number of latest data
 * @param t upon return contains time of latest data
 */
void
OpenRobotinoComThread::get_act_velocity(float &a1, float &a2, float &a3, unsigned int &seq, fawkes::Time &t)
{
	MutexLocker lock(data_mutex_);

#ifdef HAVE_OPENROBOTINO_API_1
	state_mutex_->lock();
	t = times_[active_state_];
	rec::iocontrol::remotestate::SensorState sensor_state = sensor_states_[active_state_];
	state_mutex_->unlock();

	// div by 1000 to convert from mm to m
	a1 = sensor_state.actualVelocity[0] / 1000.f;
	a2 = sensor_state.actualVelocity[1] / 1000.f;
	a3 = sensor_state.actualVelocity[2] / 1000.f;
	seq = sensor_state.sequenceNumber;
#else
	t.stamp();

	float mot_act_vel[motors_com_->numMotors()];
	motors_com_->actualVelocities(mot_act_vel);

	double odo_x = 0, odo_y = 0, odo_phi = 0;
	odom_com_->readings(&odo_x, &odo_y, &odo_phi, &seq);

	a1 = mot_act_vel[0];
	a2 = mot_act_vel[1];
	a3 = mot_act_vel[2];
#endif
}


/** Get current odometry.
 * @param x X coordinate of robot in odometry frame
 * @param y Y coordinate of robot in odometry frame
 * @param phi orientation of robot in odometry frame
 */
void
OpenRobotinoComThread::get_odometry(double &x, double &y, double &phi)
{
	MutexLocker lock(data_mutex_);

#ifdef HAVE_OPENROBOTINO_API_1
	state_mutex_->lock();
	rec::iocontrol::remotestate::SensorState sensor_state = sensor_states_[active_state_];
	state_mutex_->unlock();

	x   = sensor_state.odometryX / 1000.f;
	y   = sensor_state.odometryY / 1000.f;
	phi = deg2rad(sensor_state.odometryPhi);

#else
	unsigned int seq;
	odom_com_->readings(&x, &y, &phi, &seq);
#endif
}


/** Check if gripper is open.
 * @return true if the gripper is presumably open, false otherwise
 */
bool
OpenRobotinoComThread::is_gripper_open()
{
	MutexLocker lock(data_mutex_);

#ifdef HAVE_OPENROBOTINO_API_1
	state_mutex_->lock();
	rec::iocontrol::remotestate::SensorState sensor_state = sensor_states_[active_state_];
	state_mutex_->unlock();

	return sensor_state.isGripperOpened;
#else
	return gripper_com_->isOpened();
#endif
}


/** Set speed points for wheels.
 * @param s1 speed point for first wheel in RPM
 * @param s2 speed point for second wheel in RPM
 * @param s3 speed point for third wheel in RPM
 */
void
OpenRobotinoComThread::set_speed_points(float s1, float s2, float s3)
{
#ifdef HAVE_OPENROBOTINO_API_1
	set_state_->speedSetPoint[0] = s1;
	set_state_->speedSetPoint[1] = s2;
	set_state_->speedSetPoint[2] = s3;

	com_->setSetState(*set_state_);
#else
	float speeds[3] = { s1, s2, s3 };
	motors_com_->setSpeedSetPoints(speeds, 3);
#endif

}


/** Open or close gripper.
 * @param opened true to open gripper, false to close
 */
void
OpenRobotinoComThread::set_gripper(bool opened)
{
#ifdef HAVE_OPENROBOTINO_API_1
	set_state_->gripper_close = ! opened;
	com_->setSetState(*set_state_);
#else
	if (opened) {
		gripper_com_->open();
	} else {
		gripper_com_->close();
	}
#endif

}


void
OpenRobotinoComThread::set_bumper_estop_enabled(bool enabled)
{
#ifdef HAVE_OPENROBOTINO_API_1
	logger->log_info(name(), "%sabling bumper estop on request",
	                 msg->is_enabled() ? "En" : "Dis");
	state_->emergencyStop.isEnabled = msg->is_enabled();
#else
	logger->log_info(name(), "Setting bumper estop not supported for API2");
#endif
}
