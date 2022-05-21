
/***************************************************************************
 *  node_thread.h - ROS node handle providing thread
 *
 *  Created: Thu May 05 18:35:12 2011
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

#ifndef _PLUGINS_ROS_NODE_THREAD_H_
#define _PLUGINS_ROS_NODE_THREAD_H_

#include <aspect/aspect_provider.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <plugins/ros2/aspect/ros2_inifin.h>
#include <sys/types.h>
#include <utils/time/time.h>
#include <chrono>
#include <unistd.h>
#include <regex>
#include <rclcpp/rclcpp.hpp>

class ROS2NodeThread : public fawkes::Thread,
//                      public fawkes::BlockedTimingAspect,
                      public fawkes::LoggingAspect,
                      public fawkes::ConfigurableAspect,
                      public fawkes::ClockAspect,
                      public fawkes::AspectProviderAspect
{
public:
	ROS2NodeThread();
	virtual ~ROS2NodeThread();

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
	bool cfg_async_spinning_;
	unsigned int cfg_async_num_threads_;

	// TODO: the following probably needs to be a fawkes::LockPtr to use the one given by the aspect
	rclcpp::Node::SharedPtr node_handle;
	fawkes::ROS2AspectIniFin ros2_aspect_inifin_;

//	rclcpp::executors::MultiThreadedExecutor executor;
//	rclcpp::executors::SingleThreadedExecutor executor;
	rclcpp::executors::MultiThreadedExecutor * mult_executor;
  std::chrono::duration<double> period = std::chrono::milliseconds(10);
};

#endif
