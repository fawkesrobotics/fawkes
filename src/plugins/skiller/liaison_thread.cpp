
/***************************************************************************
 *  liaison_thread.cpp - Fawkes Skiller: Liaison Thread
 *
 *  Created: Mon Feb 18 10:24:46 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <plugins/skiller/liaison_thread.h>
#include <plugins/skiller/exec_thread.h>

#include <core/threading/barrier.h>
#include <interfaces/object.h>
#include <interfaces/skiller.h>
#include <interfaces/navigator.h>
#include <interfaces/gamestate.h>

#include <cstring>

using namespace fawkes;

/** @class SkillerLiaisonThread <plugins/skiller/liaison_thread.h>
 * Skiller Liaison Thread.
 * This threads connects the skill module to the Fawkes main loop. It gathers
 * data from the interface and passes commands to the real skill execution
 * thread.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param liaison_exec_barrier Barrier used to synchronize liaison and exec thread
 */
SkillerLiaisonThread::SkillerLiaisonThread(fawkes::Barrier *liaison_exec_barrier)
  : Thread("SkillerLiaisonThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
  __liaison_exec_barrier = liaison_exec_barrier;
}


/** Destructor. */
SkillerLiaisonThread::~SkillerLiaisonThread()
{
  bbio_add_interface_create_type("ObjectPositionInterface");
}


/** Set execution thread.
 * @param set skiller execution thread
 */
void
SkillerLiaisonThread::set_execthread(SkillerExecutionThread *set)
{
  __exec_thread = set;
}


/** Clean up when init failed.
 * You may only call this from init(). Never ever call it from anywhere
 * else!
 */
void
SkillerLiaisonThread::init_failure_cleanup()
{
  try {
    if ( skiller )    blackboard->close(skiller);
    if ( wm_ball )    blackboard->close(wm_ball);
    if ( wm_pose )    blackboard->close(wm_pose);
    //if ( wm_ball_w )  blackboard->close(wm_ball_w);
    if ( navigator )  blackboard->close(navigator);
    //if ( wm_pose_w )  blackboard->close(wm_pose_w);
    if ( gamestate )  blackboard->close(gamestate);
  } catch (...) {
    // we really screwed up, can't do anything about it, ignore error, logger is
    // initialized since this method is only called from init() which is only called if
    // all aspects had been initialized successfully
    logger->log_error(name(), "Really screwed up while finalizing, aborting cleanup. "
		              "Fawkes is no longer in a clean state. Restart!");
  }
}


void
SkillerLiaisonThread::init()
{
  wm_ball      = NULL;
  wm_pose      = NULL;
  navigator    = NULL;
  //wm_ball_w    = NULL;
  //wm_pose_w    = NULL;
  gamestate    = NULL;

  try {
    skiller   = blackboard->open_for_writing<SkillerInterface>("Skiller");
    wm_ball   = blackboard->open_for_reading<ObjectPositionInterface>("WM Ball");
    //wm_ball_w = blackboard->open_for_writing<ObjectPositionInterface>("WM Ball");
    wm_pose   = blackboard->open_for_reading<ObjectPositionInterface>("WM Pose");
    //wm_pose_w = blackboard->open_for_writing<ObjectPositionInterface>("WM Pose");
    navigator = blackboard->open_for_reading<NavigatorInterface>("Navigator");
    gamestate = blackboard->open_for_reading<GameStateInterface>("WM GameState");
    std::list<ObjectPositionInterface *> *obs_lst = blackboard->open_all_of_type_for_reading<ObjectPositionInterface>("WM Obstacles");
    for (std::list<ObjectPositionInterface *>::iterator i = obs_lst->begin(); i != obs_lst->end(); ++i) {
      wm_obstacles.push_back(*i);
    }
    delete obs_lst;
  } catch (Exception &e) {
    init_failure_cleanup();
    e.append("SkillerLiaisonThread::init() failed");
    throw;
  }

  // we only watch create events, since we never ever close an interface while
  // running, we only open newly created ones. We have memory an do not want
  // "oscillating" open/close loops
  blackboard->register_observer(this, BlackBoard::BBIO_FLAG_CREATED);

  // We want to know if our reader leaves and closes the interface
  bbil_add_reader_interface(skiller);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_READER);
}


void
SkillerLiaisonThread::finalize()
{
  blackboard->unregister_listener(this);

  blackboard->close(skiller);
  blackboard->close(wm_ball);
  //blackboard->close(wm_ball_w);
  blackboard->close(wm_pose);
  //blackboard->close(wm_pose_w);
  blackboard->close(navigator);
  blackboard->close(gamestate);
}


void
SkillerLiaisonThread::bb_interface_created(const char *type, const char *id) throw()
{
  if ( strncmp(id, "WM Obstacle", strlen("WM Obstacle")) == 0 ) {
    // It's a new obstacle in WM, open it
    try {
      ObjectPositionInterface *oi = blackboard->open_for_reading<ObjectPositionInterface>(id);
      wm_obstacles.push_back_locked(oi);
    } catch (Exception &e) {
      logger->log_error("SkillerLiaisonThread", "Tried to open new %s interface instance "
			"'%s', failed, ignoring this interface.", type, id);
      logger->log_error("SkillerLiaisonThread", e);
    }
  }
}


void
SkillerLiaisonThread::bb_interface_reader_removed(Interface *interface,
						  unsigned int instance_serial) throw()
{
  __exec_thread->skiller_reader_removed(instance_serial);
}


void
SkillerLiaisonThread::loop()
{
  //wm_ball_w->set_world_x(wm_ball_w->world_x() + 1);
  //wm_ball_w->set_visible(false);
  //wm_ball_w->write();

  /* Can be used for debugging if worldmodel/localization is not available
  wm_pose_w->set_world_x(wm_pose_w->world_x() + 0.1);
  wm_pose_w->set_world_y(wm_pose_w->world_y() + 0.1);
  wm_pose_w->write();
  */

  wm_ball->read();
  wm_pose->read();
  navigator->read();
  gamestate->read();

  wm_obstacles.lock();
  for (wm_obs_it = wm_obstacles.begin(); wm_obs_it != wm_obstacles.end(); ++wm_obs_it) {
    (*wm_obs_it)->read();
  }
  wm_obstacles.unlock();

  __liaison_exec_barrier->wait();
}
