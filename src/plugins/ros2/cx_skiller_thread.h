/***************************************************************************
 *  imu_thread.h - Thread to publish IMU data to ROS
 *
 *  Created: Oct 2023
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

#ifndef _PLUGINS_ROS2_IMU_THREAD_H_
#define _PLUGINS_ROS2_IMU_THREAD_H_

#include "skill_node.h"

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/SkillerInterface.h>
#include <plugins/ros2/aspect/ros2.h>

#include <rclcpp/rclcpp.hpp>
class ROS2CXSkillerThread : public fawkes::Thread,
                            public fawkes::ConfigurableAspect,
                            public fawkes::LoggingAspect,
                            public fawkes::ROS2Aspect,
                            public fawkes::BlackBoardAspect
{
public:
	ROS2CXSkillerThread();
	virtual ~ROS2CXSkillerThread();

	virtual void init();
	virtual void finalize();

private:
	std::shared_ptr<fawkes::SkillNode> skill_node_;
	fawkes::SkillerInterface          *skiller_iface_;
};

#endif /* PLUGINS_ROS_CXSkiller_THREAD_H__ */
