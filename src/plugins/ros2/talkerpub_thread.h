
/***************************************************************************
 *  talkerpub_thread.h - Publish talker messages via ROS
 *
 *  Created: Thu May 05 18:48:40 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_ROS2_TALKERPUB_THREAD_H_
#define _PLUGINS_ROS2_TALKERPUB_THREAD_H_

#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <plugins/ros2/aspect/ros2.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/publisher.hpp>

class ROS2TalkerPubThread : public fawkes::Thread,
                           public fawkes::BlockedTimingAspect,
                           public fawkes::LoggingAspect,
                           public fawkes::ConfigurableAspect,
                           public fawkes::ClockAspect,
                           public fawkes::ROS2Aspect
{
public:
	ROS2TalkerPubThread();
	virtual ~ROS2TalkerPubThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

#endif
