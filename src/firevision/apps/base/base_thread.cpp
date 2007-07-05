
/***************************************************************************
 *  base_thread.cpp - FireVision Base Thread
 *
 *  Created: Tue May 29 16:41:50 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <apps/base/base_thread.h>
#include <apps/base/aquisition_thread.h>

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <fvutils/system/camargp.h>
#include <cams/factory.h>

#include <apps/base/aquisition_thread.h>
#include <utils/logging/logger.h>

#include <unistd.h>

/** @class FvBaseThread <apps/base/base_thread.h>
 * FireVision base thread.
 * This implements the functionality of the FvBasePlugin.
 * @author Tim Niemueller
 */

/** Constructor. */
FvBaseThread::FvBaseThread()
  : Thread("FvBaseThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR),
    VisionMasterAspect(this)
{
  // default to 30 seconds
  _aqt_timeout = 20;
  timeout_mutex = new Mutex();
}


/** Destructor. */
FvBaseThread::~FvBaseThread()
{
  delete timeout_mutex;
}


void
FvBaseThread::init()
{
}


void
FvBaseThread::finalize()
{
  logger->log_debug("FvBaseThread", "Finalizing");
  aquisition_threads.lock();
  for (ait = aquisition_threads.begin(); ait != aquisition_threads.end(); ++ait) {
    logger->log_debug("FvBaseThread", "Cancelling aquisition thread %s", (*ait).second->name());
    (*ait).second->cancel();
    logger->log_debug(Thread::current_thread()->name(), "Joining aquisition thread %s", (*ait).second->name());
    (*ait).second->join();
    delete (*ait).second;
  }
  aquisition_threads.clear();
  aquisition_threads.unlock();
}


/** Thread loop. */
void
FvBaseThread::loop()
{
  if ( ! timeout_mutex->tryLock() ) {
    // a timeout has happened

    aquisition_threads.lock();
    timeout_aqts.lock();

    while ( ! timeout_aqts.empty() ) {
      const char *id = timeout_aqts.front();

      FvAquisitionThread *aqt = aquisition_threads[id];

      if ( aqt->empty() ) {
	logger->log_debug(name(), "Stopping thread %s", aqt->name());
	aquisition_threads.erase(id);
	aqt->cancel();
	aqt->join();
	delete aqt;
      } else {
	logger->log_debug(name(), "Aquisition thread %s not empty", aqt->name());
      }

      timeout_aqts.pop();
    }
    timeout_aqts.unlock();
    aquisition_threads.unlock();
    timeout_mutex->unlock();
  } else {
    timeout_mutex->unlock();
  }
}


/** Get vision master.
 * @return vision master
 */
VisionMaster *
FvBaseThread::vision_master()
{
  return this;
}


Camera *
FvBaseThread::register_for_camera(const char *camera_string, Thread *thread, bool raw)
{
  Camera *c;

  timeout_mutex->lock();
  aquisition_threads.lock();
  timeout_aqts.lock();

  logger->log_info(name(), "Thread '%s' register for camera '%s'", thread->name(), camera_string);

  CameraArgumentParser *cap = new CameraArgumentParser(camera_string);
  try {
    std::string id = cap->cam_type() + ":" + cap->cam_id();
    if ( aquisition_threads.find(id) != aquisition_threads.end() ) {
      // this camera has already been loaded
      c = aquisition_threads[id]->camera_instance(raw);
      try {
	aquisition_threads[id]->add_thread(thread, raw);
      } catch (Exception &e) {
	e.append("Could not add thread to '%s'", aquisition_threads[id]->name());
	delete c;
	delete cap;
 	aquisition_threads.unlock();
	timeout_aqts.unlock();
	timeout_mutex->unlock();
	throw;
      }
    } else {
      Camera *cam = CameraFactory::instance(cap);
      try {
	cam->open();
	cam->start();
      } catch (Exception &e) {
	delete cam;
	delete cap;
	e.append("Could not open or start camera");
	throw;
      }

      FvAquisitionThread *aqt = new FvAquisitionThread(this, logger, id.c_str(),
						       cam, _aqt_timeout);

      c = aqt->camera_instance(raw);
      aqt->add_thread(thread, raw);

      aquisition_threads[id] = aqt;
      aqt->start();

      logger->log_info(name(), "Aquisition thread '%s' started for thread '%s' and camera '%s'",
		       aqt->name(), thread->name(), id.c_str());

    }
  } catch (UnknownCameraTypeException &e) {
    delete cap;
    e.append("FvBaseVisionMaster: could not instantiate camera");
    aquisition_threads.unlock();
    timeout_aqts.unlock();
    timeout_mutex->unlock();
    throw;
  } catch (Exception &e) {
    delete cap;
    e.append("FvBaseVisionMaster: could not open or start camera");
    aquisition_threads.unlock();
    timeout_aqts.unlock();
    timeout_mutex->unlock();
    throw;
  }

  delete cap;

  timeout_aqts.unlock();
  aquisition_threads.unlock();
  timeout_mutex->unlock();
  return c;
}

void
FvBaseThread::unregister_thread(Thread *thread)
{
  aquisition_threads.lock();
  for (ait = aquisition_threads.begin(); ait != aquisition_threads.end(); ++ait) {
    if ( (*ait).second->has_thread(thread) ) {
      (*ait).second->remove_thread(thread);
    }
  }
  aquisition_threads.unlock();
}


/** Timeout signal receiver for aquisition threads.
 * This method is used by FvAquisitionThread to signal a timeout with empty queues.
 * If the aquisition thread is empty (which may change if the aquisition thread
 * calls this method while a new thread is added and the lock was aquired there
 * before we could aquire it) it is stopped and deleted.
 * @param id id of the aquisition thread
 */
void
FvBaseThread::aqt_timeout(const char *id)
{  
  timeout_mutex->tryLock();
  // unlocked in loop()
  logger->log_debug(name(), "Aqusition thread %s timed out", id);
  timeout_aqts.lock();
  logger->log_debug(name(), "Pushing Aqusition thread %s", id);
  timeout_aqts.push(id);
  logger->log_debug(name(), "Aqusition thread REALLY %s timed out", id);
  timeout_aqts.unlock();

}
