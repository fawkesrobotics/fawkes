
/***************************************************************************
 *  gazebo_inifin.cpp - Fawkes GazeboAspect initializer/finalizer
 *
 *  Created: Fri Aug 24 09:26:04 2012
 *  Author  Bastian Klingen, Frederik Zwilling (2013)
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

#include <plugins/gazebo/aspect/gazebo_inifin.h>
#include <core/threading/thread_finalizer.h>
#include <stdio.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class GazeboAspectIniFin <plugins/gazebo/aspect/gazebo_inifin.h>
 * GazeboAspect initializer/finalizer.
 * This initializer/finalizer will provide the Gazebo node handle to
 * threads with the GazeboAspect.
 * @author Bastian Klingen, Frederik Zwilling
 */

/** Constructor. */
GazeboAspectIniFin::GazeboAspectIniFin()
  : AspectIniFin("GazeboAspect")
{
}

/** Initialize
 * @param thread thread
 */
void
GazeboAspectIniFin::init(Thread *thread)
{
  GazeboAspect *gazebo_thread;
  gazebo_thread = dynamic_cast<GazeboAspect *>(thread);
  if (gazebo_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "GazeboAspect, but RTTI says it "
					  "has not. ", thread->name());
  }
  if (! __gazebonode) {
    throw CannotInitializeThreadException("Gazebo node handle has not been set.");
  }

  gazebo_thread->init_GazeboAspect(__gazebonode, __gazebo_world_node);
}

/** Finalize
 * @param thread thread
 */
void
GazeboAspectIniFin::finalize(Thread *thread)
{
  GazeboAspect *gazebo_thread;
  gazebo_thread = dynamic_cast<GazeboAspect *>(thread);
  if (gazebo_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"GazeboAspect, but RTTI says it "
					"has not. ", thread->name());
  }
  gazebo_thread->finalize_GazeboAspect();
}


/** Set the Gazebo node handle to use for aspect initialization.
 * (used for robot specific communication)
 * @param gazebonode Gazebo node handle to pass to threads with GazeboAspect.
 */
void
GazeboAspectIniFin::set_gazebonode(gazebo::transport::NodePtr gazebonode)
{
  __gazebonode = gazebonode;
}
/** Set the Gazebo node handle to use for aspect initialization.
 * (used for robot independent or world changing communication)
 * @param gazebo_world_node Gazebo node handle to pass to threads with GazeboAspect.
 */
void
GazeboAspectIniFin::set_gazebo_world_node(gazebo::transport::NodePtr gazebo_world_node)
{
  __gazebo_world_node = gazebo_world_node;
}

} // end namespace fawkes
