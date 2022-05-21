
/***************************************************************************
 *  ros.cpp - ROS aspect for Fawkes
 *
 *  Created: Thu May 05 15:53:31 2011
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

#include <plugins/ros2/aspect/ros2.h>
#include <rclcpp/rclcpp.hpp>

namespace fawkes {

/** @class ROS2Aspect <plugins/ros/aspect/ros.h>
 * Thread aspect to get access to a ROS2 node handle.
 * Give this aspect to your thread to interact with the central ROS2
 * node handle.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @var fawkes:LockPtr<ros::NodeHandle> ROS2Aspect::rosnode
 * Central ROS2 node handle. Make sure you use proper locking in your
 * application when using the class, or chaos and havoc will come upon you.
 */

/** Constructor. */
ROS2Aspect::ROS2Aspect()
{
	add_aspect("ROS2Aspect");
}

/** Virtual empty destructor. */
ROS2Aspect::~ROS2Aspect()
{
}

/** Init ROS2 aspect.
 * This set the ROS2 node handle.
 * It is guaranteed that this is called for an ROS2 Thread before start
 * is called (when running regularly inside Fawkes).
 * @param node_handle ROS2 node handle
 */
void
ROS2Aspect::init_ROS2Aspect(rclcpp::Node::SharedPtr node_handle)
{
	this->node_handle = node_handle;
//        cfg_tf_prefix_ = config->get_string_or_default("/ros2/tf/tf_prefix", "");
//        cfg_tf_prefix_exclusions_ = config->get_strings_or_defaults("/ros2/tf/tf_prefix_exclusions", std::vector<std::string>());
//
//        tf_prefix_enabled_ = false;
//        if (cfg_tf_prefix_ != "") {
//                tf_prefix_enabled_ = true;
//        }
//
//        if (cfg_tf_prefix_ == "$HOSTNAME") {
//                HostInfo hinfo;
//                // namespace must not contain characters other than alphanumerics, '_', or '/'
//                cfg_tf_prefix_ = hinfo.short_name();
//                std::regex tf_prefix_pattern("[^A-Za-z0-9_]");
//
//                // write the results to an output iterator
//                cfg_tf_prefix_ = std::regex_replace(cfg_tf_prefix_,
//                                                    tf_prefix_pattern, "") + std::string("_");
//        }
}

///** Add the tf_prefix in case it is not to be excluded
// * @param frame_id the frame_id to add the prefix to
// */
//void
//ROS2Aspect::add_tf_prefix(std::string &frame_id)
//{
//        if (tf_prefix_enabled_ == true && std::find(cfg_tf_prefix_exclusions_.begin(), cfg_tf_prefix_exclusions_.end(), frame_id) == cfg_tf_prefix_exclusions_.end()) {
//                // cfg_tf_prefix_exclusions_ does not contain ts.header.frame_id so we can add the prefix.
//		frame_id.insert(0, cfg_tf_prefix_);
//	}
//}

/** Finalize ROS2 aspect.
 * This clears the ROS2 node handle.
 */
void
ROS2Aspect::finalize_ROS2Aspect()
{
	//rosnode.clear();
}

} // end namespace fawkes
