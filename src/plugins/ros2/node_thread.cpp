
/***************************************************************************
 *  node_thread.cpp - ROS2 node handle providing Thread
 *
 *  Created: Thu May 05 18:37:08 2011
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

#include "node_thread.h"

#include <utils/system/hostinfo.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>

using namespace fawkes;

/** @class ROS2NodeThread "node_thread.h"
 * ROS2 node handle thread.
 * This thread maintains a ROS2 node which can be used by other
 * threads and is provided via the ROSAspect.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
ROS2NodeThread::ROS2NodeThread()
: Thread("ROS2NodeThread", Thread::OPMODE_CONTINUOUS),
//  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP),
  AspectProviderAspect(&ros2_aspect_inifin_)
{
}

/** Destructor. */
ROS2NodeThread::~ROS2NodeThread()
{
}

void
ROS2NodeThread::init()
{
	cfg_async_spinning_ = false;
	try {
		cfg_async_spinning_ = config->get_bool("/ros2/async-spinning");
	} catch (Exception &e) {
	} // ignored, use default

	cfg_async_num_threads_ = 4;
	try {
		cfg_async_num_threads_ = config->get_uint("/ros2/async-num-threads");
	} catch (Exception &e) {
	} // ignored, use default

	int         argc      = 1;
	const char *argv[]    = {"fawkes"};
		rclcpp::init(argc, (char **)argv);

	std::string node_name = config->get_string_or_default("/ros2/node-name", "fawkes");

	if (node_name == "$HOSTNAME") {
		HostInfo hinfo;
		node_name = hinfo.short_name();
	}

        std::string ros_namespace = config->get_string_or_default("/ros2/namespace", "");

	if (ros_namespace == "$HOSTNAME") {
		HostInfo hinfo;
                // namespace must not contain characters other than alphanumerics, '_', or '/'
                ros_namespace = hinfo.short_name();
                std::regex ros_namespace_pattern("[^A-Za-z0-9_]");
             
                // write the results to an output iterator
                ros_namespace = std::regex_replace(ros_namespace,
                                                   ros_namespace_pattern, "");

		ros_namespace = std::string("/") + ros_namespace;
	}

	if (node_handle == NULL || !node_handle->get_node_options().context()->is_valid()) {
		node_handle = rclcpp::Node::make_shared(node_name, ros_namespace);
	} else {
		logger->log_warn(name(), "ROS2 node already initialized");
	}

	ros2_aspect_inifin_.set_node_handle(node_handle);

	bool yield_before_execute = true;

	mult_executor = new rclcpp::executors::MultiThreadedExecutor(
				rclcpp::ExecutorOptions(), 2u, yield_before_execute);

	if (cfg_async_spinning_) {
		mult_executor->add_node(node_handle);
		std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, mult_executor));
	}
}

void
ROS2NodeThread::finalize()
{
	rclcpp::shutdown();
	ros2_aspect_inifin_.set_node_handle(node_handle);
}

void
ROS2NodeThread::loop()
{
	if (!cfg_async_spinning_) {
		rclcpp::spin_some(node_handle);
    usleep(100000);
	}
}
