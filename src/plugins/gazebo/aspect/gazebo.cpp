
/***************************************************************************
 *  gazebo.cpp - Gazebo aspect for Fawkes
 *
 *  Created: Fri Aug 24 09:24:31 2012
 *  Author Bastian Klingen, Frederik Zwilling
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

#include <plugins/gazebo/aspect/gazebo.h>
#include <stdio.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class GazeboAspect <plugins/gazebo/aspect/gazebo.h>
 * Thread aspect to get access to a Gazebo node handle.
 * Give this aspect to your thread to interact with the central Gazebo
 * node handle.
 * You can also control the simulation by sending messages
 * via the provided publishers
 *
 * @ingroup Aspects
 * @author Bastian Klingen, Frederik Zwilling
 */

/** @var fawkes:LockPtr<gazebo::NodeHandle> GazeboAspect::gazebonode
 * Central Gazebo node handle. Make sure you use proper locking in your
 * application when using the class, or chaos and havoc will come upon you.
 */

/** Constructor. */
GazeboAspect::GazeboAspect()
{
  add_aspect("GazeboAspect");
}


/** Virtual empty destructor. */
GazeboAspect::~GazeboAspect()
{
}


/** Init Gazebo aspect.
 * This set the Gazebo node handle.
 * It is guaranteed that this is called for an Gazebo Thread before start
 * is called (when running regularly inside Fawkes).
 * @param gazebonode Gazebo node handle
 */
void
GazeboAspect::init_GazeboAspect(gazebo::transport::NodePtr gazebonode , gazebo::transport::NodePtr gazebo_world_node)
{
  this->gazebonode = gazebonode;
  this->gazebo_world_node = gazebo_world_node;
}

/** Finalize Gazebo aspect.
 * This clears the Gazebo node handle.
 */
void
GazeboAspect::finalize_GazeboAspect()
{
  gazebonode.reset();
}

} // end namespace fawkes
