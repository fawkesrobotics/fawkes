
/***************************************************************************
 *  thread.cpp - Fawkes ball position logger - for demonstration
 *
 *  Created: Thu Jan 24 17:03:56 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include <plugins/examples/ballposlog/thread.h>
#include <interfaces/ObjectPositionInterface.h>

/** @class BallPosLogThread thread.h "thread.h"
 * Main thread of ball position logger plugin.
 * @author Tim Niemueller
 */

using namespace fawkes;

/** Constructor.
 */
BallPosLogThread::BallPosLogThread()
  : Thread("BallPosLogThread",
	   Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(
        BlockedTimingAspect::WAKEUP_HOOK_THINK)
{
}


/** Destructor. */
BallPosLogThread::~BallPosLogThread()
{
}


void
BallPosLogThread::init()
{
  wm_ball_interface      = NULL;
  try {
    wm_ball_interface =
      blackboard->open_for_reading<ObjectPositionInterface>("WM Ball");
    log_level = (Logger::LogLevel)config->
                get_uint("/ballposlog/log_level");
  } catch (Exception &e) {
    blackboard->close(wm_ball_interface);
    throw;
  }
}


void
BallPosLogThread::finalize()
{
  blackboard->close(wm_ball_interface);
}


void
BallPosLogThread::loop()
{
  wm_ball_interface->read();

  logger->log(log_level, "BallPosLog",
	      "Ball is at global (x,y) = (%f,%f)",
	      wm_ball_interface->world_x(),
	      wm_ball_interface->world_y());
}
