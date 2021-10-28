
/***************************************************************************
 *  image_thread.cpp - Thread to exchange point clouds
 *
 *  Created: Tue Apr 10 22:12:27 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_ROS2_IMAGE_THREAD_H_
#define _PLUGINS_ROS2_IMAGE_THREAD_H_

#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>
#include <image_transport/image_transport.hpp>
#include <plugins/ros2/aspect/ros2.h>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

#include <list>
#include <queue>

namespace firevision {
class SharedMemoryImageBuffer;
}

class ROS2ImagesThread : public fawkes::Thread,
                        public fawkes::ClockAspect,
                        public fawkes::LoggingAspect,
                        public fawkes::ConfigurableAspect,
                        public fawkes::BlockedTimingAspect,
                        public fawkes::ROS2Aspect
{
public:
	ROS2ImagesThread();
	virtual ~ROS2ImagesThread();

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
	void update_images();
	void get_sets(std::set<std::string> &missing_images, std::set<std::string> &unbacked_images);

private:
	/// @cond INTERNALS
	typedef struct
	{
		image_transport::Publisher           pub;
		sensor_msgs::msg::Image                   msg;
		fawkes::Time                         last_sent;
		firevision::SharedMemoryImageBuffer *img;
	} PublisherInfo;
	/// @endcond
	std::map<std::string, PublisherInfo> pubs_;

	image_transport::ImageTransport *it_;
	fawkes::Time *                   last_update_;
	fawkes::Time *                   now_;
};

#endif
