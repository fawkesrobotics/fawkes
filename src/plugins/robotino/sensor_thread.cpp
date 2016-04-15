
/***************************************************************************
 *  sensor_thread.cpp - Robotino sensor thread
 *
 *  Created: Sun Nov 13 15:35:24 2011
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

#include "sensor_thread.h"
#include "com_thread.h"

#include <interfaces/BatteryInterface.h>
#include <interfaces/RobotinoSensorInterface.h>
#include <interfaces/IMUInterface.h>

using namespace fawkes;

/** @class RobotinoSensorThread "sensor_thread.h"
 * Robotino sensor hook integration thread.
 * This thread integrates into the Fawkes main loop at the SENSOR hook and
 * writes new sensor data.
 * @author Tim Niemueller
 */

/// taken from Robotino API2 DistanceSensorImpl.hpp
const std::vector<std::pair<double, double> > VOLTAGE_TO_DIST_DPS =
	{
		{0.3 , 0.41},	{0.39, 0.35},	{0.41, 0.30},	{0.5 , 0.25},	{0.75, 0.18},
		{0.8 , 0.16},	{0.95, 0.14},	{1.05, 0.12},	{1.3 , 0.10},	{1.4 , 0.09},
		{1.55, 0.08},	{1.8 , 0.07},	{2.35, 0.05},	{2.55, 0.04}
	};


/** Constructor.
 * @param com_thread communication thread to trigger for writing data
 */
RobotinoSensorThread::RobotinoSensorThread(RobotinoComThread *com_thread)
	: Thread("RobotinoSensorThread", Thread::OPMODE_WAITFORWAKEUP),
	  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
	com_ = com_thread;
}


void
RobotinoSensorThread::init()
{
	cfg_enable_gyro_ = config->get_bool("/hardware/robotino/gyro/enable");
	cfg_imu_iface_id_ = config->get_string("/hardware/robotino/gyro/interface_id");

	batt_if_ = NULL;
	sens_if_ = NULL;
	imu_if_ = NULL;

	batt_if_ = blackboard->open_for_writing<BatteryInterface>("Robotino");
	sens_if_ = blackboard->open_for_writing<RobotinoSensorInterface>("Robotino");

	if (cfg_enable_gyro_) {
		imu_if_ = blackboard->open_for_writing<IMUInterface>(cfg_imu_iface_id_.c_str());
	}
}


void
RobotinoSensorThread::finalize()
{
	blackboard->close(sens_if_);
	blackboard->close(batt_if_);
	blackboard->close(imu_if_);
}

void
RobotinoSensorThread::loop()
{
	process_sensor_msgs();

	RobotinoComThread::SensorData data;
	if (com_->get_data(data)) {
		sens_if_->set_mot_velocity(data.mot_velocity);
		sens_if_->set_mot_position(data.mot_position);
		sens_if_->set_mot_current(data.mot_current);
		sens_if_->set_bumper(data.bumper);
		sens_if_->set_bumper_estop_enabled(data.bumper_estop_enabled);
		sens_if_->set_digital_in(data.digital_in);
		sens_if_->set_digital_out(data.digital_out);
		sens_if_->set_analog_in(data.analog_in);
		update_distances(data.ir_voltages);
		sens_if_->write();
		
		batt_if_->set_voltage(data.bat_voltage);
		batt_if_->set_current(data.bat_current);
		batt_if_->set_absolute_soc(data.bat_absolute_soc);
		batt_if_->write();
		
		if (cfg_enable_gyro_) {
			if (data.imu_enabled) {
				imu_if_->set_angular_velocity(data.imu_angular_velocity);
				imu_if_->set_angular_velocity_covariance(data.imu_angular_velocity_covariance);
				imu_if_->set_orientation(data.imu_orientation);
				imu_if_->write();
			} else {
				if (fabs(data.imu_angular_velocity[0] + 1.) > 0.00001) {
					imu_if_->set_linear_acceleration(0, -1.);
					imu_if_->set_angular_velocity(0, -1.);
					imu_if_->set_angular_velocity(2,  0.);
					imu_if_->set_orientation(0, -1.);
					imu_if_->write();
				}
			}
		}
	}
}


void
RobotinoSensorThread::process_sensor_msgs()
{
	// process command messages
	while (! sens_if_->msgq_empty()) {
		if (RobotinoSensorInterface::SetBumperEStopEnabledMessage *msg =
		    sens_if_->msgq_first_safe(msg))
		{
			com_->set_bumper_estop_enabled(msg->is_enabled());
		} else if (RobotinoSensorInterface::SetDigitalOutputMessage *msg =
		    sens_if_->msgq_first_safe(msg))
		{
			try {
				com_->set_digital_output(msg->digital_out(), msg->is_enabled());
			} catch (Exception &e) {
				logger->log_warn(name(), e);
			}
		}
		sens_if_->msgq_pop();
	} // while sensor msgq
}

void
RobotinoSensorThread::update_distances(float *voltages)
{
	float dist_m[NUM_IR_SENSORS];
	const size_t num_dps = VOLTAGE_TO_DIST_DPS.size();

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

			const double lv = VOLTAGE_TO_DIST_DPS[j  ].first;
			const double rv = VOLTAGE_TO_DIST_DPS[j+1].first;

			if ((voltages[i] >= lv) && (voltages[i] < rv)) {
				const double ld = VOLTAGE_TO_DIST_DPS[j  ].second;
				const double rd = VOLTAGE_TO_DIST_DPS[j+1].second;

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
