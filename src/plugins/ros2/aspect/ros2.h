
/***************************************************************************
 *  ros.h - ROS2 aspect for Fawkes
 *
 *  Created: Thu May 05 16:01:34 2011
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

#ifndef _PLUGINS_ROS2_ASPECT_ROS2_H_
#define _PLUGINS_ROS2_ASPECT_ROS2_H_

#include <aspect/aspect.h>
#include <core/utils/lockptr.h>
#include <rclcpp/rclcpp.hpp>

namespace fawkes {

class ROS2AspectIniFin;

class ROS2Aspect : public virtual Aspect
{
	friend ROS2AspectIniFin;

public:
	ROS2Aspect();
	virtual ~ROS2Aspect();

protected:
	rclcpp::Node::SharedPtr node_handle;

private:
	void init_ROS2Aspect(rclcpp::Node::SharedPtr node_handle);
	void finalize_ROS2Aspect();
};

} // end namespace fawkes

#endif
