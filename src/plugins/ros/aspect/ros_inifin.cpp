
/***************************************************************************
 *  ros_inifin.cpp - Fawkes ROSAspect initializer/finalizer
 *
 *  Created: Thu May 05 16:03:34 2011
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

#include <plugins/ros/aspect/ros_inifin.h>
#include <core/threading/thread_finalizer.h>
#include <ros/node_handle.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ROSAspectIniFin <plugins/ros/aspect/ros_inifin.h>
 * ROSAspect initializer/finalizer.
 * This initializer/finalizer will provide the ROS node handle to
 * threads with the ROSAspect.
 * @author Tim Niemueller
 */

/** Constructor. */
ROSAspectIniFin::ROSAspectIniFin()
  : AspectIniFin("ROSAspect")
{
}

void
ROSAspectIniFin::init(Thread *thread)
{
  ROSAspect *ros_thread;
  ros_thread = dynamic_cast<ROSAspect *>(thread);
  if (ros_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "ROSAspect, but RTTI says it "
					  "has not. ", thread->name());
  }
  if (! __rosnode) {
    throw CannotInitializeThreadException("ROS node handle has not been set.");
  }

  ros_thread->init_ROSAspect(__rosnode);
}

void
ROSAspectIniFin::finalize(Thread *thread)
{
  ROSAspect *ros_thread;
  ros_thread = dynamic_cast<ROSAspect *>(thread);
  if (ros_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"ROSAspect, but RTTI says it "
					"has not. ", thread->name());
  }
  ros_thread->finalize_ROSAspect();
}


/** Set the ROS node handle to use for aspect initialization.
 * @param rosnode ROS node handle to pass to threads with ROSAspect.
 */
void
ROSAspectIniFin::set_rosnode(LockPtr<ros::NodeHandle> rosnode)
{
  __rosnode = rosnode;
}

} // end namespace fawkes
