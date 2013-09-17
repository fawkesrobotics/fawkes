
/***************************************************************************
 *  gazebo.h - Gazebo aspect for Fawkes
 *
 *  Created: Fri Aug 24 09:25:22 2012
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

#ifndef __PLUGINS_GAZEBO_ASPECT_GAZEBO_H_
#define __PLUGINS_GAZEBO_ASPECT_GAZEBO_H_

#include <aspect/aspect.h>

// from Gazebo
#include <gazebo/transport/Node.hh>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class GazeboAspect : public virtual Aspect
{
  friend class GazeboAspectIniFin;

 public:
  GazeboAspect();
  virtual ~GazeboAspect();

 protected:
  gazebo::transport::NodePtr gazebonode;
  gazebo::transport::NodePtr gazebo_world_node;
  gazebo::transport::PublisherPtr visual_publisher, model_publisher, request_publisher, light_publisher;

 private:
  void init_GazeboAspect(gazebo::transport::NodePtr gazebonode, gazebo::transport::NodePtr gazebo_world_node,
			 gazebo::transport::PublisherPtr visual_publisher, gazebo::transport::PublisherPtr model_publisher,
			 gazebo::transport::PublisherPtr request_publisher, gazebo::transport::PublisherPtr light_publisher);
  void finalize_GazeboAspect();
};

} // end namespace fawkes

#endif
