
/***************************************************************************
 *  wm_thread.cpp - Fawkes WorldModel Plugin Thread
 *
 *  Created: Fri Jun 29 11:56:48 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <plugins/worldmodel/wm_thread.h>

#include <interfaces/object.h>

/** @class WorldModelThread <plugins/worldmodel/wm_thread.h>
 * Main thread of worldmodel plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 */
WorldModelThread::WorldModelThread()
  : Thread("WorldModelThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}


/** Destructor. */
WorldModelThread::~WorldModelThread()
{
}


/** Clean up when init failed.
 * You may only call this from init(). Never ever call it from anywhere
 * else!
 */
void
WorldModelThread::init_failure_cleanup()
{
  try {
    if ( wm_ball_interface )  blackboard->close(wm_ball_interface);
    if ( wm_pose_interface )  blackboard->close(wm_pose_interface);
    if ( in_ball_interfaces ) {
      for (opii = in_ball_interfaces->begin(); opii != in_ball_interfaces->end(); ++opii) {
	blackboard->close(*opii);
      }
      delete in_ball_interfaces;
    }
    if ( in_opp_interfaces ) {
      for (opii = in_opp_interfaces->begin(); opii != in_opp_interfaces->end(); ++opii) {
	blackboard->close(*opii);
      }
      delete in_opp_interfaces;
    }
  } catch (...) {
    // we really screwed up, can't do anything about it, ignore error, logger is
    // initialized since this method is only called from init() which is only called if
    // all aspects had been initialized successfully
    logger->log_error(name(), "Really screwed up while finalizing, aborting cleanup. "
		              "Fawkes is no longer in a clean state. Restart!");
  }
}


void
WorldModelThread::init()
{
  wm_ball_interface      = NULL;
  wm_pose_interface      = NULL;
  wm_opp_interfaces      = NULL;

  in_ball_interfaces     = NULL;
  in_opp_interfaces = NULL;

  try {
    in_ball_interfaces = blackboard->open_all_of_type_for_reading<ObjectPositionInterface>("Ball ");
    for (opii = in_ball_interfaces->begin(); opii != in_ball_interfaces->end(); ++opii) {
      // for each incoming ball interface, register for notifications of writer death
      // blackboard->register_listener(listener, BBEL_FLAG_WRITER);
    }

    wm_ball_interface = blackboard->open_for_writing<ObjectPositionInterface>("WM Ball");
    wm_pose_interface = blackboard->open_for_writing<ObjectPositionInterface>("WM Pose");
    wm_opp_interfaces = new std::list<ObjectPositionInterface *>();

  } catch (Exception &e) {
    init_failure_cleanup();
    e.append("WorldModel::init() failed");
    throw;
  }
}


void
WorldModelThread::finalize()
{
  for (opii = in_ball_interfaces->begin(); opii != in_ball_interfaces->end(); ++opii) {
    blackboard->close(*opii);    
  }
  delete in_ball_interfaces;

  blackboard->close(wm_ball_interface);
  blackboard->close(wm_pose_interface);

  for (wm_opp_interfaces->begin(); opii != wm_opp_interfaces->end(); ++opii) {
    blackboard->close(*opii);    
  }
  delete wm_opp_interfaces;
}


void
WorldModelThread::loop()
{
  /*
   * 1. Collect data (i.e. update local reader interfaces, collect data from network thread)
   * 2. Fusion network data (if available)
   * 3. Do calculations (to build up a consistent world model carry out general calculations
   *                     like the old best interceptor here at a central place
   * 4. Update written interfaces with new data
   *
   */
}
