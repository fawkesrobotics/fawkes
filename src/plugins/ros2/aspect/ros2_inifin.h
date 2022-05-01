
/***************************************************************************
 *  ros_inifin.h - Fawkes ROSAspect initializer/finalizer
 *
 *  Created: Thu May 05 16:02:26 2011
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

#ifndef _PLUGINS_ROS2_ASPECT_ROS_INIFIN_H_
#define _PLUGINS_ROS2_ASPECT_ROS_INIFIN_H_

#include <aspect/inifins/inifin.h>
#include <plugins/ros2/aspect/ros2.h>
#include <rclcpp/rclcpp.hpp>


namespace fawkes {

class ROS2AspectIniFin : public AspectIniFin
{
public:
	ROS2AspectIniFin();

	virtual void init(Thread *thread);
	virtual void finalize(Thread *thread);

	void set_node_handle(rclcpp::Node::SharedPtr node_handle);

private:
	rclcpp::Node::SharedPtr node_handle_;
	std::string tf_prefix_;
};

} // end namespace fawkes

#endif
