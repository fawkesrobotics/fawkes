
/***************************************************************************
 *  liaison_thread.cpp - Fawkes Skiller: Liaison Thread
 *
 *  Created: Mon Feb 18 10:24:46 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <plugins/skiller/liaison_thread.h>

#include <core/threading/barrier.h>
#include <interfaces/object.h>

#include <cstring>

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
SkillerLiaisonThread::SkillerLiaisonThread(Barrier *liaison_exec_barrier)
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


/** Clean up when init failed.
 * You may only call this from init(). Never ever call it from anywhere
 * else!
 */
void
SkillerLiaisonThread::init_failure_cleanup()
{
  try {
    if ( wm_ball_interface )  blackboard->close(wm_ball_interface);
    if ( wm_ball_interface_w )  blackboard->close(wm_ball_interface_w);
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
  wm_ball_interface      = NULL;
  wm_pose_interface      = NULL;

  try {
    wm_ball_interface = blackboard->open_for_reading<ObjectPositionInterface>("WM Ball");
    wm_ball_interface_w = blackboard->open_for_writing<ObjectPositionInterface>("WM Ball");
    wm_pose_interface = blackboard->open_for_reading<ObjectPositionInterface>("WM Pose");
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
}


void
SkillerLiaisonThread::finalize()
{
  blackboard->close(wm_ball_interface);
  blackboard->close(wm_pose_interface);
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
SkillerLiaisonThread::loop()
{
  wm_ball_interface_w->read();
  wm_ball_interface_w->set_world_x(wm_ball_interface_w->world_x() + 1);
  wm_ball_interface_w->write();

  wm_ball_interface->read();
  wm_pose_interface->read();
  wm_obstacles.lock();
  for (wm_obs_it = wm_obstacles.begin(); wm_obs_it != wm_obstacles.end(); ++wm_obs_it) {
    (*wm_obs_it)->read();
  }
  wm_obstacles.unlock();

  __liaison_exec_barrier->wait();
}
