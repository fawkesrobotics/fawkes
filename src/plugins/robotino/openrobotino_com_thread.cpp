
/***************************************************************************
 *  com_thread.cpp - Robotino com thread
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
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <utils/math/angle.h>
#include <utils/time/wait.h>
#include <tf/types.h>

#include <interfaces/BatteryInterface.h>
#include <interfaces/RobotinoSensorInterface.h>
#include <interfaces/IMUInterface.h>

#include <unistd.h>

using namespace fawkes;

#define NUM_IR_SENSORS 9


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
	cfg_hostname_ = config->get_string("/hardware/robotino/hostname");
	cfg_quit_on_disconnect_ = config->get_bool("/hardware/robotino/quit_on_disconnect");
	cfg_enable_gyro_ = config->get_bool("/hardware/robotino/gyro/enable");
	cfg_imu_iface_id_ = config->get_string("/hardware/robotino/gyro/interface_id");
	cfg_sensor_update_cycle_time_ =
		config->get_uint("/hardware/robotino/sensor_update_cycle_time");
	cfg_gripper_enabled_ = config->get_bool("/hardware/robotino/gripper/enable_gripper");

	batt_if_ = NULL;
	sens_if_ = NULL;
	imu_if_ = NULL;

	batt_if_ = blackboard->open_for_writing<BatteryInterface>("Robotino");
	sens_if_ = blackboard->open_for_writing<RobotinoSensorInterface>("Robotino");

	if (cfg_enable_gyro_) {
		imu_if_ = blackboard->open_for_writing<IMUInterface>(cfg_imu_iface_id_.c_str());
	}

	// taken from Robotino API2 DistanceSensorImpl.hpp
	voltage_to_dist_dps_.push_back(std::make_pair(0.3 , 0.41));
	voltage_to_dist_dps_.push_back(std::make_pair(0.39, 0.35));
	voltage_to_dist_dps_.push_back(std::make_pair(0.41, 0.30));
	voltage_to_dist_dps_.push_back(std::make_pair(0.5 , 0.25));
	voltage_to_dist_dps_.push_back(std::make_pair(0.75, 0.18));
	voltage_to_dist_dps_.push_back(std::make_pair(0.8 , 0.16));
	voltage_to_dist_dps_.push_back(std::make_pair(0.95, 0.14));
	voltage_to_dist_dps_.push_back(std::make_pair(1.05, 0.12));
	voltage_to_dist_dps_.push_back(std::make_pair(1.3 , 0.10));
	voltage_to_dist_dps_.push_back(std::make_pair(1.4 , 0.09));
	voltage_to_dist_dps_.push_back(std::make_pair(1.55, 0.08));
	voltage_to_dist_dps_.push_back(std::make_pair(1.8 , 0.07));
	voltage_to_dist_dps_.push_back(std::make_pair(2.35, 0.05));
	voltage_to_dist_dps_.push_back(std::make_pair(2.55, 0.04));

#ifdef HAVE_OPENROBOTINO_API_1
	statemem_ =  new rec::sharedmemory::SharedMemory<rec::iocontrol::robotstate::State>
		(rec::iocontrol::robotstate::State::sharedMemoryKey);
	state_ = statemem_->getData();
	state_mutex_ = new Mutex();
	set_state_ = new rec::iocontrol::remotestate::SetState();
	set_state_->gripper_isEnabled = cfg_gripper_enabled_;
	active_state_ = 0;
#endif

	if (imu_if_) {
		// Assume that the gyro is the CruizCore XG1010 and thus set data
		// from datasheet
		imu_if_->set_linear_acceleration(0, -1.);
		imu_if_->set_angular_velocity_covariance(8, deg2rad(0.1));
		imu_if_->write();
	}

	data_mutex_  = new Mutex();
	new_data_    = false;
	last_seqnum_ = 0;
	time_wait_   = new TimeWait(clock, cfg_sensor_update_cycle_time_);

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
	delete data_mutex_;
	delete time_wait_;
	blackboard->close(sens_if_);
	blackboard->close(batt_if_);
	blackboard->close(imu_if_);
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
		process_sensor_msgs();

		MutexLocker lock(data_mutex_);
#ifdef HAVE_OPENROBOTINO_API_1
		process_sensor_state();
#else
		process_com();
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
OpenRobotinoComThread::process_sensor_msgs()
{
	// process command messages
	while (! sens_if_->msgq_empty()) {
		if (RobotinoSensorInterface::SetBumperEStopEnabledMessage *msg =
		    sens_if_->msgq_first_safe(msg))
		{
#ifdef HAVE_OPENROBOTINO_API_1
			logger->log_info(name(), "%sabling motor on request",
			                 msg->is_enabled() ? "En" : "Dis");
			state_->emergencyStop.isEnabled = msg->is_enabled();
#else
			logger->log_info(name(), "Setting emergency stop not yet supported for API2");
#endif
		}
		sens_if_->msgq_pop();
	} // while sensor msgq
}


void
OpenRobotinoComThread::process_sensor_state()
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
		sens_if_->set_mot_velocity(sensor_state.actualVelocity);
		sens_if_->set_mot_position(sensor_state.actualPosition);
		sens_if_->set_mot_current(sensor_state.motorCurrent);
		sens_if_->set_bumper(sensor_state.bumper);
		sens_if_->set_bumper_estop_enabled(state_->emergencyStop.isEnabled);
		sens_if_->set_digital_in(sensor_state.dIn);
		sens_if_->set_analog_in(sensor_state.aIn);
		if (cfg_enable_gyro_) {
			if (state_->gyro.port == rec::serialport::UNDEFINED) {
				if (fabs(imu_if_->angular_velocity(0) + 1.) > 0.00001) {
					imu_if_->set_angular_velocity(0, -1.);
					imu_if_->set_angular_velocity(2,  0.);
					imu_if_->set_orientation(0, -1.);
				}
			} else {
				imu_if_->set_angular_velocity(0, 0.);
				imu_if_->set_angular_velocity(2, state_->gyro.rate);

				tf::Quaternion q = tf::create_quaternion_from_yaw(state_->gyro.angle);
				imu_if_->set_orientation(0, q.x());
				imu_if_->set_orientation(1, q.y());
				imu_if_->set_orientation(2, q.z());
				imu_if_->set_orientation(3, q.w());
			}
		}

		update_distances(sensor_state.distanceSensor);

		batt_if_->set_voltage(roundf(sensor_state.voltage * 1000.));
		batt_if_->set_current(roundf(sensor_state.current));

		// 21.0V is empty, 26.0V is empty, from OpenRobotino lcdd
		float soc = (sensor_state.voltage - 21.0f) / 5.f;
		soc = std::min(1.f, std::max(0.f, soc));

		batt_if_->set_absolute_soc(soc);
	}
#endif
}


void
OpenRobotinoComThread::process_com()
{
#ifdef HAVE_OPENROBOTINO_API_2
	com_->processComEvents();

	double odo_x = 0, odo_y = 0, odo_phi = 0;
	unsigned int odo_seq = 0;

	odom_com_->readings(&odo_x, &odo_y, &odo_phi, &odo_seq);

	if (odo_seq != last_seqnum_) {  
		new_data_ = true;
		last_seqnum_ = odo_seq;

		unsigned int mot_num = motors_com_->numMotors();
		float mot_act_vel[mot_num];
		int   mot_act_pos[mot_num];
		float mot_currents[mot_num];
		motors_com_->actualVelocities(mot_act_vel);
		motors_com_->actualPositions(mot_act_pos);
		motors_com_->motorCurrents(mot_currents);

		bool bumper = bumper_com_->value();

		unsigned int digin_num = digital_inputs_com_->numDigitalInputs();
		int digin_readings[digin_num];
		bool digin_bools[digin_num];
		digital_inputs_com_->values(digin_readings);
		for (unsigned int i = 0; i < digin_num; ++i)  digin_bools[i] = (digin_readings[i] != 0);

		unsigned int anlgin_num = analog_inputs_com_->numAnalogInputs();
		float anlgin_readings[anlgin_num];
		analog_inputs_com_->values(anlgin_readings);

		unsigned int dist_num = distances_com_->numDistanceSensors();
		// the distance calculation from API2 uses a max value of 0.41,
		// which breaks the previous behavior of 0.0 for "nothing"
		// therefore use our API1 conversion routine
		//float dist_distances[dist_num];
		//distances_com_->distances(dist_distances);
		float dist_voltages[dist_num];
		distances_com_->voltages(dist_voltages);

		if (anlgin_num != sens_if_->maxlenof_analog_in()) {
			logger->log_warn(name(), "Different number of analog inputs: %u vs. %u",
			                 anlgin_num, sens_if_->maxlenof_analog_in());
		}

		if (digin_num != sens_if_->maxlenof_digital_in()) {
			logger->log_warn(name(), "Different number of digital inputs: %u vs. %u",
			                 digin_num, sens_if_->maxlenof_digital_in());
		}

		if (dist_num != sens_if_->maxlenof_distance()) {
			logger->log_warn(name(), "Different number of distances: %u vs. %u",
			                 dist_num, sens_if_->maxlenof_distance());
		}

		float pow_current = power_com_->current() * 1000.; // A -> mA
		float pow_voltage = power_com_->voltage() * 1000.; // V -> mV

		float gyro_angle  = gyroscope_com_->angle();
		float gyro_rate   = gyroscope_com_->rate();

		// update sensor values in interface
		sens_if_->set_mot_velocity(mot_act_vel);
		sens_if_->set_mot_position(mot_act_pos);
		sens_if_->set_mot_current(mot_currents);
		sens_if_->set_bumper(bumper);
		//sens_if_->set_bumper_estop_enabled(???);
		sens_if_->set_digital_in(digin_bools);
		sens_if_->set_analog_in(anlgin_readings);
		if (cfg_enable_gyro_) {
			imu_if_->set_angular_velocity(0, 0.);
			imu_if_->set_angular_velocity(2, gyro_rate);

			tf::Quaternion q = tf::create_quaternion_from_yaw(gyro_angle);
			imu_if_->set_orientation(0, q.x());
			imu_if_->set_orientation(1, q.y());
			imu_if_->set_orientation(2, q.z());
			imu_if_->set_orientation(3, q.w());
		}

		//sens_if_->set_distance(dist_distances);
		update_distances(dist_voltages);

		batt_if_->set_voltage(roundf(pow_voltage));
		batt_if_->set_current(roundf(pow_current));

		// 22.0V is empty, 24.5V is full, this is just a guess
		float soc = (power_com_->voltage() - 22.0f) / 2.5f;
		soc = std::min(1.f, std::max(0.f, soc));
		batt_if_->set_absolute_soc(soc);
	}
#endif
}


/** Trigger writes of blackboard interfaces.
 * This is meant to be called by the sensor thread so that writes to the
 * blackboard happen in the sensor acquisition hook.
 */
void
OpenRobotinoComThread::update_bb_sensor()
{
	MutexLocker lock(data_mutex_);
	if (new_data_) {
		batt_if_->write();
		sens_if_->write();
		if (imu_if_)  imu_if_->write();
		new_data_ = false;
	}
}

void
OpenRobotinoComThread::update_distances(float *voltages)
{
	float dist_m[NUM_IR_SENSORS];
	const size_t num_dps = voltage_to_dist_dps_.size();

	for (int i = 0; i < NUM_IR_SENSORS; ++i) {
		dist_m[i] = 0.;
		// find the two enclosing data points

		for (size_t j = 0; j < num_dps - 1; ++j) {
			// This determines two points, l(eft) and r(ight) that are
			// defined by voltage (x coord) and distance (y coord). We
			// assume a linear progression between two adjacent points,
			// i.e. between l and r. We then do the following:
			// 1. Find two adjacent voltage values lv and rv where
			//    the voltage lies inbetween
			// 2. Interpolate by calculating the line parameters
			//    m = dd/dv, x = voltage - lv and b = ld.
			// cf. http://www.acroname.com/robotics/info/articles/irlinear/irlinear.html

			const double lv = voltage_to_dist_dps_[j  ].first;
			const double rv = voltage_to_dist_dps_[j+1].first;

			if ((voltages[i] >= lv) && (voltages[i] < rv)) {
				const double ld = voltage_to_dist_dps_[j  ].second;
				const double rd = voltage_to_dist_dps_[j+1].second;

				double dv = rv - lv;
				double dd = rd - ld;

				// Linear interpolation between 
				dist_m[i] = (dd / dv) * (voltages[i] - lv) + ld;
				break;
			}
		}
	}

	sens_if_->set_distance(dist_m);
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
