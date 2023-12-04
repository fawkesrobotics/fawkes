/***************************************************************************
 *  navgraph_thread.h - Thread to relay between Navgraph Blackboard Interfaces
 *                      and ROS2
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

#ifndef _PLUGINS_ROS2_NAVGRAPH_THREAD_H_
#define _PLUGINS_ROS2_NAVGRAPH_THREAD_H_

#include "navgraph_node.h"

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/NavGraphGeneratorInterface.h>
#include <interfaces/NavGraphWithMPSGeneratorInterface.h>
#include <plugins/ros2/aspect/ros2.h>

#include <rclcpp/rclcpp.hpp>
class ROS2NavgraphThread : public fawkes::Thread,
                           public fawkes::ConfigurableAspect,
                           public fawkes::LoggingAspect,
                           public fawkes::ROS2Aspect,
                           public fawkes::BlackBoardAspect
{
public:
	ROS2NavgraphThread();
	virtual ~ROS2NavgraphThread();

	virtual void init();
	virtual void finalize();

private:
	std::shared_ptr<fawkes::ROS2NavgraphNode>  navgraph_node_;
	fawkes::NavGraphGeneratorInterface        *navgraph_interface_;
	fawkes::NavGraphWithMPSGeneratorInterface *navgraph_mps_interface_;
};

#endif /* PLUGINS_ROS2_NAVGRAPH_THREAD_H__ */