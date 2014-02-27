
/***************************************************************************
 *  gazebo_inifin.h - Fawkes GazeboAspect initializer/finalizer
 *
 *  Created: Fri Aug 24 09:26:58 2012
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

#ifndef __PLUGINS_GAZEBO_ASPECT_GAZEBO_INIFIN_H_
#define __PLUGINS_GAZEBO_ASPECT_GAZEBO_INIFIN_H_

#include <aspect/inifins/inifin.h>
#include <plugins/gazebo/aspect/gazebo.h>

// from Gazebo
#include <gazebo/transport/Node.hh>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class GazeboAspectIniFin : public AspectIniFin
{
 public:
  GazeboAspectIniFin();

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);

  //setters for the node_thread
  void set_gazebonode(gazebo::transport::NodePtr gazebonode);
  void set_gazebo_world_node(gazebo::transport::NodePtr gazebo_world_node);

 private:
  gazebo::transport::NodePtr __gazebonode;
  gazebo::transport::NodePtr __gazebo_world_node;

};

} // end namespace fawkes

#endif
