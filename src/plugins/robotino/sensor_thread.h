
/***************************************************************************
 *  sensor_thread.h - Robotino sensor thread
 *
 *  Created: Sun Nov 13 15:33:04 2011
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_ROBOTINO_SENSOR_THREAD_H_
#define __PLUGINS_ROBOTINO_SENSOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <string>
#include <vector>

class RobotinoComThread;
class RobotinoActThread;

namespace fawkes {
	class BatteryInterface;
	class RobotinoSensorInterface;
	class IMUInterface;
}


class RobotinoSensorThread
: public fawkes::Thread,
	public fawkes::BlockedTimingAspect,
	public fawkes::LoggingAspect,
	public fawkes::ClockAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::BlackBoardAspect
{
	friend RobotinoActThread;
 public:
	RobotinoSensorThread(RobotinoComThread *com_thread);

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // methods
	void process_sensor_msgs();
	void update_distances(float *voltages);

	// Voltage to distance data points
	static const std::vector<std::pair<double, double> > voltage_to_dist_dps_;


 private: // members
	RobotinoComThread *com_;

	bool            cfg_enable_gyro_;
	std::string     cfg_imu_iface_id_;

	fawkes::BatteryInterface        *batt_if_;
	fawkes::RobotinoSensorInterface *sens_if_;
	fawkes::IMUInterface            *imu_if_;

};


#endif
