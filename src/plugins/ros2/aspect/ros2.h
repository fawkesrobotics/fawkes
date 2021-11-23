
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

//#include <aspect/aspect.h>
#include <aspect/configurable.h>
#include <core/utils/lockptr.h>
#include <rclcpp/rclcpp.hpp>
#include <utils/system/hostinfo.h>
#include <regex>



namespace fawkes {

class ROS2AspectIniFin;

class ROS2Aspect : public virtual Aspect

{
	friend ROS2AspectIniFin;

public:
	ROS2Aspect();
	virtual ~ROS2Aspect();
	void add_tf_prefix(std::string & frame_id);

protected:
	rclcpp::Node::SharedPtr node_handle;

private:
	void init_ROS2Aspect(rclcpp::Node::SharedPtr node_handle);
	void finalize_ROS2Aspect();
//	bool tf_prefix_enabled_;
//	std::string cfg_tf_prefix_;
//	std::vector<std::string> cfg_tf_prefix_exclusions_;
};

} // end namespace fawkes

#endif
