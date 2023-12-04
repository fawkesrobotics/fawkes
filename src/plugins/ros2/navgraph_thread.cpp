/***************************************************************************
 *  navgraph_thread.cpp - Thread to relay between Navgraph Blackboard Interfaces
 *                        and ROS2
 *
 *  Created: Dec 2023
 *  Copyright  2023  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

#include "navgraph_thread.h"

using namespace fawkes;

/** @class ROS2NavgraphThread "navgraph_thread.h"
 * Thread to relay between Navgraph Blackboard Interfaces and ROS2
 * @author Daniel Swoboda
 */

/** Constructor. */
ROS2NavgraphThread::ROS2NavgraphThread()
: Thread("ROS2CXSkillerThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
ROS2NavgraphThread::~ROS2NavgraphThread()
{
}

/** Initialize the thread.
 * Open the blackboard interface and advertise the ROS topic.
 * Register as listener for the blackboard interface.
 */
void
ROS2NavgraphThread::init()
{
	navgraph_interface_ =
	  blackboard->open_for_reading<NavGraphGeneratorInterface>("NavGraphGeneratorInterface",
	                                                           "/navgraph-generator");
	navgraph_mps_interface_ = blackboard->open_for_reading<NavGraphWithMPSGeneratorInterface>(
	  "NavGraphWithMPSGeneratorInterface", "/navgraph-generator-mps");
	navgraph_node_ = std::make_shared<ROS2NavgraphNode>("FawkesNavgraphNode",
	                                                    std::chrono::nanoseconds(5000),
	                                                    navgraph_interface_,
	                                                    navgraph_mps_interface_);
	navgraph_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

	blackboard->register_listener(navgraph_node_.get());
}

/** Close all interfaces and ROS handles. */
void
ROS2NavgraphThread::finalize()
{
	blackboard->unregister_listener(navgraph_node_.get());
	navgraph_node_.reset();
	blackboard->close(navgraph_interface_);
	blackboard->close(navgraph_mps_interface_);
}
