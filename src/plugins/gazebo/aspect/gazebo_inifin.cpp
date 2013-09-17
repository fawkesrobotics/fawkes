
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

  gazebo_thread->init_GazeboAspect(__gazebonode, __gazebo_world_node, __visual_publisher,
				   __model_publisher, __request_publisher, __light_publisher);
}

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
 * @param gazebonode Gazebo node handle to pass to threads with GazeboAspect.
 */
void
GazeboAspectIniFin::set_gazebonode(gazebo::transport::NodePtr gazebonode)
{
  __gazebonode = gazebonode;
}
void
GazeboAspectIniFin::set_gazebo_world_node(gazebo::transport::NodePtr gazebo_world_node)
{
  __gazebo_world_node = gazebo_world_node;
}
void
GazeboAspectIniFin::set_visual_publisher(gazebo::transport::PublisherPtr publisher)
{
  __visual_publisher = publisher;
}
void
GazeboAspectIniFin::set_model_publisher(gazebo::transport::PublisherPtr publisher)
{
  __model_publisher = publisher;
}
void
GazeboAspectIniFin::set_request_publisher(gazebo::transport::PublisherPtr publisher)
{
  __request_publisher = publisher;
}
void
GazeboAspectIniFin::set_light_publisher(gazebo::transport::PublisherPtr publisher)
{
  __light_publisher = publisher;
}

} // end namespace fawkes
