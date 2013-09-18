
/***************************************************************************
 *  node_thread.h - Gazebo node handle providing thread
 *
 *  Created: Fri Aug 24 11:04:04 2012
 *  Author  Bastian Klingen, Frederik Zwilling
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_GAZEBO_NODE_THREAD_H_
#define __PLUGINS_GAZEBO_NODE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blocked_timing.h>
#include <aspect/aspect_provider.h>
#include <plugins/gazebo/aspect/gazebo_inifin.h>
#include <utils/time/time.h>
#include <string.h>

#include <sys/types.h>

namespace gazebo {
  namespace transport {
    class Node;
  }
}

class GazeboNodeThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::AspectProviderAspect
{
 public:
  GazeboNodeThread();
  virtual ~GazeboNodeThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  //Node for communication to gazebo-robot-plugins
  gazebo::transport::NodePtr  __gazebonode;
  //Node to control the gazebo world (e.g. spawn visual objects)
  gazebo::transport::NodePtr  __gazebo_world_node;
  //Publisher to send Messages:
  gazebo::transport::PublisherPtr __visual_publisher, __model_publisher, __request_publisher, __light_publisher;

  fawkes::GazeboAspectIniFin  __gazebo_aspect_inifin;

  //channel of a specified robot for the gazebo node communication
  std::string robot_channel, world_name;
};

#endif
