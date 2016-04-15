/***************************************************************************
 *  com_thread.h - Robotino com thread
 *
 *  Created: Thu Sep 11 11:43:42 2014
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

#ifndef __PLUGINS_ROBOTINO_OPENROBOTINO_COM_THREAD_H_
#define __PLUGINS_ROBOTINO_OPENROBOTINO_COM_THREAD_H_

#include "com_thread.h"
#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <utils/time/time.h>

#ifdef HAVE_OPENROBOTINO_API_1
#  include <rec/robotino/com/Com.h>
namespace rec {
	namespace sharedmemory {
		template<typename SharedType> class SharedMemory;
	}
	namespace iocontrol {
		namespace robotstate {
			class State;
		}
		namespace remotestate {
			class SetState;
		}
	}
}
#else
namespace rec {
	namespace robotino {
		namespace api2 {
			class Com;
			class AnalogInputArray;
			class Bumper;
			class DigitalInputArray;
			class DistanceSensorArray;
			class ElectricalGripper;
			class Gyroscope;
			class MotorArray;
			class Odometry;
			class PowerManagement;
		}
	}
}
#endif

namespace fawkes {
	class Mutex;
	class Clock;
	class TimeWait;
}

class OpenRobotinoComThread
: public RobotinoComThread,
#ifdef HAVE_OPENROBOTINO_API_1
	public rec::robotino::com::Com,
#endif
	public fawkes::ConfigurableAspect
{
 public:
	OpenRobotinoComThread();
	virtual ~OpenRobotinoComThread();

	virtual void init();
	virtual void once();
	virtual void loop();
	virtual void finalize();

	virtual bool is_connected();

	virtual void set_gripper(bool opened);
	virtual bool is_gripper_open();
	virtual void set_speed_points(float s1, float s2, float s3);
	virtual void get_act_velocity(float &a1, float &a2, float &a3, unsigned int &seq, fawkes::Time &t);
	virtual void get_odometry(double &x, double &y, double &phi);

	virtual void reset_odometry();
	virtual void set_bumper_estop_enabled(bool enabled);
	virtual void set_motor_accel_limits(float min_accel, float max_accel);
	virtual void set_digital_output(unsigned int digital_out, bool enable);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
#ifdef HAVE_OPENROBOTINO_API_1
	using rec::robotino::com::Com::sensorState;
	virtual void updateEvent();
#endif
	void process_api_v1();
	void process_api_v2();
	
 private:
	std::string     cfg_hostname_;
	bool            cfg_quit_on_disconnect_;
	unsigned int    cfg_sensor_update_cycle_time_;
	bool            cfg_gripper_enabled_;
	bool            cfg_enable_gyro_;

	fawkes::TimeWait *time_wait_;
	unsigned int      last_seqnum_;

#ifdef HAVE_OPENROBOTINO_API_1
	rec::robotino::com::Com *com_;
	fawkes::Mutex *state_mutex_;
	unsigned int active_state_;
	rec::iocontrol::remotestate::SensorState sensor_states_[2];
	fawkes::Time times_[2];

	rec::sharedmemory::SharedMemory<rec::iocontrol::robotstate::State> *statemem_;
	rec::iocontrol::robotstate::State *state_;

	rec::iocontrol::remotestate::SetState *set_state_;

#else
	rec::robotino::api2::Com                    *com_;
	rec::robotino::api2::AnalogInputArray       *analog_inputs_com_;
	rec::robotino::api2::Bumper                 *bumper_com_;
	rec::robotino::api2::DigitalInputArray      *digital_inputs_com_;
	rec::robotino::api2::DistanceSensorArray    *distances_com_;
	rec::robotino::api2::ElectricalGripper      *gripper_com_;
	rec::robotino::api2::Gyroscope              *gyroscope_com_;
	rec::robotino::api2::MotorArray             *motors_com_;
	rec::robotino::api2::Odometry               *odom_com_;
	rec::robotino::api2::PowerManagement        *power_com_;
#endif
};


#endif

