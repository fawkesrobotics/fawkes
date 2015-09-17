/***************************************************************************
 *  clock_thread.h - Thread to publish clock to ROS
 *
 *  Created: Sun Jul 12 16:14:41 2015
 *  Copyright  2015  Tim Niemueller
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

#include "clock_thread.h"
#include <rosgraph_msgs/Clock.h>
#include <utils/time/wait.h>

using namespace fawkes;

/** @class RosClockThread "clock_thread.h"
 * Thread to publish clock to ROS.
 * @author Tim Niemueller
 */

/** Constructor. */
RosClockThread::RosClockThread()
	: Thread("ClockThread", Thread::OPMODE_CONTINUOUS)
{
	set_prepfin_conc_loop(true);
}

void
RosClockThread::init()
{
	cfg_freq_ = config->get_uint("/ros/clock/frequency");
	pub_ = rosnode->advertise<rosgraph_msgs::Clock>("clock", 1);
	rosnode->setParam("/use_sim_time", true);

	set_local_ = ros::Time::isSystemTime();
	if (set_local_) {
		// enable sim time
		ros::Time::setNow(ros::Time::now());
	}

	time_wait_ = new TimeWait(clock, 1000000l / cfg_freq_);
}

void
RosClockThread::finalize()
{
	rosnode->deleteParam("/use_sim_time");
  pub_.shutdown();
  delete time_wait_;
}


void
RosClockThread::publish_clock()
{
	rosgraph_msgs::Clock clock_msg;

	fawkes::Time now(clock);
	clock_msg.clock.sec  = now.get_sec();
  clock_msg.clock.nsec = now.get_usec() * 1000;

  pub_.publish(clock_msg);

  if (set_local_)  ros::Time::setNow(clock_msg.clock);
}

void
ros_clock_cb(const rosgraph_msgs::Clock::ConstPtr& msg)
{
	ros::Time::setNow(msg->clock);
}


void
RosClockThread::loop()
{
	time_wait_->mark_start();
	publish_clock();
	time_wait_->wait_systime();
}
