/***************************************************************************
 *  position_3d_thread.h - Publish 3D Position to ROS
 *
 *  Created: Wed Jul 16 17:04:42 2014
 *  Copyright  2014  Till Hofmann
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

#ifndef _PLUGINS_ROS2_POSITION_3D_THREAD_H_
#define _PLUGINS_ROS2_POSITION_3D_THREAD_H_

// TODO check includes
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>
#include <interfaces/Position3DInterface.h>
#include <plugins/ros2/aspect/ros2.h>

#include <list>

// from ROS
#include <rclcpp/rclcpp.hpp>
#include <fawkes_msgs/msg/position3_d.hpp>

class ROS2Position3DThread : public fawkes::Thread,
                            public fawkes::ConfigurableAspect,
                            public fawkes::LoggingAspect,
                            public fawkes::BlackBoardAspect,
                            public fawkes::ROS2Aspect,
                            public fawkes::BlackBoardInterfaceObserver,
                            public fawkes::BlackBoardInterfaceListener
{
public:
	ROS2Position3DThread();
	virtual ~ROS2Position3DThread();

	virtual void init();
	virtual void finalize();

	virtual void bb_interface_created(const char *type, const char *id) throw();
	virtual void bb_interface_writer_removed(fawkes::Interface *interface,
	                                         unsigned int       instance_serial) throw();
	virtual void bb_interface_reader_removed(fawkes::Interface *interface,
	                                         unsigned int       instance_serial) throw();
	virtual void bb_interface_data_refreshed(fawkes::Interface *interface) throw();

private:
	void conditional_close(fawkes::Interface *interface) throw();

private:
	rclcpp::Publisher<fawkes_msgs::msg::Position3D>::SharedPtr ros2_pub_;
	std::list<fawkes::Position3DInterface *> ifs_;
	std::string                              cfg_ros_topic_;
};

#endif
