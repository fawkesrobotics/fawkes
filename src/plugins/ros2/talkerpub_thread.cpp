
/***************************************************************************
 *  talkerpub_thread.cpp - Publish talker messages via ROS
 *
 *  Created: Thu May 05 18:50:04 2011
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

#include "talkerpub_thread.h"


using namespace fawkes;

/** @class ROS2TalkerPubThread "talkerpub_thread.h"
 * Thread to publish messages via ROS.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
ROS2TalkerPubThread::ROS2TalkerPubThread()
: Thread("ROS2TalkerPubThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

/** Destructor. */
ROS2TalkerPubThread::~ROS2TalkerPubThread()
{
}

void
ROS2TalkerPubThread::init()
{
	pub_ = node_handle->create_publisher<std_msgs::msg::String>("/chatter", 10);
}

void
ROS2TalkerPubThread::finalize()
{
	rclcpp::shutdown();
}

void
ROS2TalkerPubThread::loop()
{
	Time t;

	std_msgs::msg::String msg;
	msg.data = std::string("Hello world ") + t.str();
	pub_->publish(msg);
}
