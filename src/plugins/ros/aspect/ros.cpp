
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

#include <plugins/ros/aspect/ros.h>
#include <ros/node_handle.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ROSAspect <plugins/ros/aspect/ros.h>
 * Thread aspect to get access to a ROS node handle.
 * Give this aspect to your thread to interact with the central ROS
 * node handle.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @var fawkes:LockPtr<ros::NodeHandle> ROSAspect::rosnode
 * Central ROS node handle. Make sure you use proper locking in your
 * application when using the class, or chaos and havoc will come upon you.
 */

/** Constructor. */
ROSAspect::ROSAspect()
{
  add_aspect("ROSAspect");
}


/** Virtual empty destructor. */
ROSAspect::~ROSAspect()
{
}


/** Init ROS aspect.
 * This set the ROS node handle.
 * It is guaranteed that this is called for an ROS Thread before start
 * is called (when running regularly inside Fawkes).
 * @param rosnode ROS node handle
 */
void
ROSAspect::init_ROSAspect(LockPtr<ros::NodeHandle> rosnode)
{
  this->rosnode = rosnode;
}

/** Finalize ROS aspect.
 * This clears the ROS node handle.
 */
void
ROSAspect::finalize_ROSAspect()
{
  rosnode.clear();
}

} // end namespace fawkes
