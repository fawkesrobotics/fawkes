
/***************************************************************************
 *  ros_inifin.cpp - Fawkes ROSAspect initializer/finalizer
 *
 *  Created: Thu May 05 16:03:34 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <core/threading/thread_finalizer.h>
#include <plugins/ros2/aspect/ros2_inifin.h>

namespace fawkes {

/** @class ROS2AspectIniFin <plugins/ros2/aspect/ros2_inifin.h>
 * ROS2Aspect initializer/finalizer.
 * This initializer/finalizer will provide the ROS2 node handle to
 * threads with the ROS2Aspect.
 * @author Tim Niemueller
 */

/** Constructor. */
ROS2AspectIniFin::ROS2AspectIniFin() : AspectIniFin("ROS2Aspect")
{
}

void
ROS2AspectIniFin::init(Thread *thread)
{
	ROS2Aspect *ros2_thread;
	ros2_thread = dynamic_cast<ROS2Aspect *>(thread);
	if (ros2_thread == NULL) {
		throw CannotInitializeThreadException("Thread '%s' claims to have the "
		                                      "ROS2Aspect, but RTTI says it "
		                                      "has not. ",
		                                      thread->name());
	}
	if (!node_handle_) {
		throw CannotInitializeThreadException("ROS2 node handle has not been set.");
	}

	ros2_thread->init_ROS2Aspect(node_handle_);
}

void
ROS2AspectIniFin::finalize(Thread *thread)
{
	ROS2Aspect *ros2_thread;
	ros2_thread = dynamic_cast<ROS2Aspect *>(thread);
	if (ros2_thread == NULL) {
		throw CannotFinalizeThreadException("Thread '%s' claims to have the "
		                                    "ROS2Aspect, but RTTI says it "
		                                    "has not. ",
		                                    thread->name());
	}
	ros2_thread->finalize_ROS2Aspect();
}

/** Set the ROS2 node handle to use for aspect initialization.
 * @param node_handle ROS2 node handle to pass to threads with ROS2Aspect.
 */
void
ROS2AspectIniFin::set_node_handle(rclcpp::Node::SharedPtr node_handle)
{
	node_handle_ = node_handle;
}

} // end namespace fawkes
