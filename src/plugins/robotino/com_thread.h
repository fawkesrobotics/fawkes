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
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <utils/time/time.h>

namespace fawkes {
	class Mutex;
	class Clock;
	class TimeWait;

	class BatteryInterface;
	class RobotinoSensorInterface;
	class IMUInterface;
}

class RobotinoComThread
: public fawkes::Thread
{
 public:
	RobotinoComThread(const char *thread_name);
	virtual ~RobotinoComThread();

	virtual void update_bb_sensor() = 0;

	virtual bool is_connected() = 0;

	virtual void set_gripper(bool opened) = 0;
	virtual bool is_gripper_open() = 0;
	virtual void set_speed_points(float s1, float s2, float s3) = 0;
	virtual void get_act_velocity(float &a1, float &a2, float &a3, unsigned int &seq, fawkes::Time &t) = 0;
	virtual void get_odometry(double &x, double &y, double &phi) = 0;
	virtual void reset_odometry() = 0;
};


#endif

