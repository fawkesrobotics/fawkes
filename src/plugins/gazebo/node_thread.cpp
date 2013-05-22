
/***************************************************************************
 *  node_thread.cpp - Gazebo node handle providing Thread
 *
 *  Created: Fri Aug 24 11:04:04 2012
 *  Author  Bastian Klingen
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

#include "node_thread.h"

// from Gazebo
#include <gazebo/transport/Transport.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/gazebo_config.h>
#include <google/protobuf/message.h>

using namespace fawkes;

/** @class GazeboNodeThread "node_thread.h"
 * Gazebo node handle thread.
 * This thread maintains a Gazebo node which can be used by other
 * threads and is provided via the GazeboAspect.
 *
 * @author Bastian Klingen
 */

/** Constructor. */
GazeboNodeThread::GazeboNodeThread()
  : Thread("GazeboNodeThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP),
    AspectProviderAspect("GazeboAspect", &__gazebo_aspect_inifin)
{
}


/** Destructor. */
GazeboNodeThread::~GazeboNodeThread()
{
}


void
GazeboNodeThread::init()
{
  if(gazebo::transport::is_stopped()) {
    gazebo::transport::init();
    gazebo::transport::run();
  }
  else {
    logger->log_warn(name(), "Gazebo already running ");
  }

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  __gazebonode = node;
  __gazebonode->Init();
  __gazebo_aspect_inifin.set_gazebonode(__gazebonode);
}


void
GazeboNodeThread::finalize()
{
  __gazebonode->Fini();
  __gazebonode.reset();
  __gazebo_aspect_inifin.set_gazebonode(__gazebonode);
}


void
GazeboNodeThread::loop()
{
}
