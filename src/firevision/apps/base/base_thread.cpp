
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
  _aqt_timeout = 30;
}


/** Destructor. */
FvBaseThread::~FvBaseThread()
{
  logger->log_info("FvBaseThread", "Destroying thread %s", name());
  aquisition_threads.lock();
  for (ait = aquisition_threads.begin(); ait != aquisition_threads.end(); ++ait) {
    (*ait).second->cancel();
    (*ait).second->join();
    delete (*ait).second;
  }
  aquisition_threads.clear();
  aquisition_threads.unlock();
}


void
FvBaseThread::init()
{
}


/** Thread loop. */
void
FvBaseThread::loop()
{
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
FvBaseThread::register_for_camera(const char *camera_string, Thread *thread)
{
  Camera *c;
  aquisition_threads.lock();

  logger->log_info(name(), "Thread '%s' register for camera '%s'", thread->name(), camera_string);

  try {
    CameraArgumentParser *cap = new CameraArgumentParser(camera_string);
    std::string id = cap->cam_type() + ":" + cap->cam_id();
    if ( aquisition_threads.find(id) != aquisition_threads.end() ) {
      // this camera has already been loaded
      c = aquisition_threads[id]->camera_instance();
      try {
	aquisition_threads[id]->add_thread(thread);
      } catch (Exception &e) {
	aquisition_threads.unlock();
	e.append("Could not add thread to '%s'", aquisition_threads[id]->name());
	delete c;
	throw;
      }
    } else {
      CameraArgumentParser *cap = new CameraArgumentParser(camera_string);
      Camera *cam = CameraFactory::instance(cap);
      cam->open();
      cam->start();

      FvAquisitionThread *aqt = new FvAquisitionThread(this, logger, id.c_str(),
						       cam, _aqt_timeout);

      c = aqt->camera_instance();
      aqt->add_thread(thread);

      aqt->start();

      logger->log_info(name(), "Aquisition thread '%s' started for thread '%s' and camera '%s'",
		       aqt->name(), thread->name(), id.c_str());

      aquisition_threads[id] = aqt;
    }
  } catch (UnknownCameraTypeException &e) {
    aquisition_threads.unlock();
    e.append("FvBaseVisionMaster: could not instantiate camera");
    throw;
  } catch (Exception &e) {
    aquisition_threads.unlock();
    e.append("FvBaseVisionMaster: could not open or start camera");
    throw;
  }

  aquisition_threads.unlock();
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
  aquisition_threads.lock();
  FvAquisitionThread *aqt = aquisition_threads[id];
  if ( aqt->empty() ) {
    aquisition_threads.erase(id);
    aqt->cancel();
    aqt->join();
    delete aqt;
  }
  aquisition_threads.unlock();
}
