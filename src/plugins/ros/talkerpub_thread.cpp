
/***************************************************************************
 *  talkerpub_thread.cpp - Publish talker messages via ROS
 *
 *  Created: Thu May 05 18:50:04 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "talkerpub_thread.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace fawkes;

/** @class ROSTalkerPubThread "talkerpub_thread.h"
 * Thread to publish messages via ROS.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
ROSTalkerPubThread::ROSTalkerPubThread()
  : Thread("ROSTalkerPubThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}


/** Destructor. */
ROSTalkerPubThread::~ROSTalkerPubThread()
{
}


void
ROSTalkerPubThread::init()
{
  __pub = rosnode->advertise<std_msgs::String>("/chatter", 10);
}


void
ROSTalkerPubThread::finalize()
{
  __pub.shutdown();
}


void
ROSTalkerPubThread::loop()
{
  Time t;

  std_msgs::String msg;
  msg.data = std::string("Hello world ") + t.str();

  __pub.publish(msg);
}
