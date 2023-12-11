/***************************************************************************
 *  cx_skiller_thread.cpp - Thread to publish CXSkiller data to ROS 2
 *
 *  Created: oct 2023
 *  Copyright  2023  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
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

#include "cx_skiller_thread.h"

using namespace fawkes;

/** @class ROS2CXSkillerThread "cx_skiller_thread.h"
 * Thread to publish CXSkiller data to ROS.
 * This thread reads data from the CXSkiller blackboard interface and publishes the
 * data to ROS.
 * @author Tarik Viehmann
 */

/** Constructor. */
ROS2CXSkillerThread::ROS2CXSkillerThread()
: Thread("ROS2CXSkillerThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
ROS2CXSkillerThread::~ROS2CXSkillerThread()
{
}

/** Initialize the thread.
 * Open the blackboard interface and advertise the ROS topic.
 * Register as listener for the blackboard interface.
 */
void
ROS2CXSkillerThread::init()
{
	skiller_iface_ = blackboard->open_for_reading<SkillerInterface>("Skiller");

	std::string robot_name = config->get_string_or_default("fawkes/agent/name", "");
	std::string executor_name =
	  config->get_string_or_default("ros2/cx-skiller/executor-name", "fawkes_skiller");

	skill_node_ = std::make_shared<SkillNode>(
	  "FawkesSkillNode", robot_name, executor_name, std::chrono::nanoseconds(5000), skiller_iface_);
	skill_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
	executor->add_node(skill_node_->get_node_base_interface());

	blackboard->register_listener(skill_node_.get());
}

/** Close all interfaces and ROS handles. */
void
ROS2CXSkillerThread::finalize()
{
	blackboard->unregister_listener(skill_node_.get());
	// call destructor before closing interface to release skiller control
	executor->remove_node(skill_node_->get_node_base_interface());
	skill_node_.reset();
	blackboard->close(skiller_iface_);
}
