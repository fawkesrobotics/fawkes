
/***************************************************************************
 *  joint_thread.h - Thread to publish JointStates to ROS
 *
 *  Created: Wed Sep 25 18:27:26 2013
 *  Copyright  2013  Till Hofmann
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

#ifndef _PLUGINS_ROS2_JOINT_THREAD_H_
#define _PLUGINS_ROS2_JOINT_THREAD_H_

// TODO check includes
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>
#include <interfaces/JointInterface.h>
#include <plugins/ros2/aspect/ros2.h>

#include <list>

// from ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class ROS2JointThread : public fawkes::Thread,
                       public fawkes::LoggingAspect,
                       public fawkes::BlackBoardAspect,
                       public fawkes::ROS2Aspect,
                       public fawkes::BlackBoardInterfaceObserver,
                       public fawkes::BlackBoardInterfaceListener
{
public:
	ROS2JointThread();
	virtual ~ROS2JointThread();

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
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ros2_pub_;
	std::list<fawkes::JointInterface *> ifs_;
};

#endif
