
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
#include <apps/base/aqt_vision_threads.h>

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/threading/barrier.h>
#include <fvutils/system/camargp.h>
#include <cams/factory.h>

#include <apps/base/aquisition_thread.h>
#include <utils/logging/logger.h>
#include <aspect/vision.h>

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
  aqt_barrier = new Barrier(1);
}


/** Destructor. */
FvBaseThread::~FvBaseThread()
{
  delete aqt_barrier;
}


void
FvBaseThread::init()
{
}


void
FvBaseThread::finalize()
{
  aqts.lock();
  for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
    (*ait).second->cancel();
    (*ait).second->join();
    delete (*ait).second;
  }
  aqts.clear();
  aqts.unlock();
}


/** Thread loop. */
void
FvBaseThread::loop()
{
  aqts.lock();

  // Wakeup all cyclic aquisition threads and wait for them
  for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
    if ( (*ait).second->aqtmode() == FvAquisitionThread::AqtCyclic ) {
      //logger->log_error(name(), "Waking Thread %s", (*ait).second->name());
      (*ait).second->wakeup(aqt_barrier);
    }
  }

  aqt_barrier->wait();
  
  // Check for aqt timeouts
  for (ait = aqts.begin(); ait != aqts.end();) {
    if ( (*ait).second->_vision_threads->empty() &&
	 ((*ait).second->_vision_threads->empty_time() > _aqt_timeout) ) {
      
      logger->log_info(name(), "Aquisition thread %s timed out, destroying",
		       (*ait).second->name());

      (*ait).second->cancel();
      (*ait).second->join();
      delete (*ait).second;
      aqts.erase(ait++);
    } else {
      ++ait;
    }
  }

  for (stit = started_threads.begin(); stit != started_threads.end();) {

    // if the thread is registered in that aqt mark it running
    (*stit).second->_vision_threads->set_thread_running((*stit).first);

    if ( (*stit).second->_vision_threads->has_cyclic_thread() ) {
      if ((*stit).second->aqtmode() != FvAquisitionThread::AqtCyclic ) {
	logger->log_info(name(), "Switching aquisition thread %s to cyclic mode (%s)",
			 (*stit).second->name(), Thread::current_thread()->name());

	(*stit).second->cancel();
	(*stit).second->join();
	(*stit).second->set_aqtmode(FvAquisitionThread::AqtCyclic);
	(*stit).second->start();
      }
    } else if ((*stit).second->aqtmode() != FvAquisitionThread::AqtContinuous ) {
      logger->log_info(name(), "Switching aquisition thread %s to continuous mode",
		       (*stit).second->name());
      (*stit).second->cancel();
      (*stit).second->join();
      (*stit).second->set_aqtmode(FvAquisitionThread::AqtContinuous);
      (*stit).second->start();
    }

    started_threads.erase( stit++ );
  }

  // Re-create barrier as necessary after _adding_ threads
  unsigned int num_cyclic_threads = 0;
  for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
    if ( (*ait).second->_vision_threads->has_cyclic_thread() ) {
      ++num_cyclic_threads;
    }
  }
  cond_recreate_barrier(num_cyclic_threads);

  aqts.unlock();
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

  aqts.lock();

  logger->log_info(name(), "Thread '%s' registers for camera '%s'", thread->name(), camera_string);

  VisionAspect *vision_thread = dynamic_cast<VisionAspect *>(thread);
  if ( vision_thread == NULL ) {
    throw TypeMismatchException("Thread is not a vision thread");
  }

  CameraArgumentParser *cap = new CameraArgumentParser(camera_string);
  try {
    std::string id = cap->cam_type() + ":" + cap->cam_id();
    if ( aqts.find(id) != aqts.end() ) {
      // this camera has already been loaded
      c = aqts[id]->camera_instance(raw,
				    (vision_thread->vision_thread_mode() ==
				     VisionAspect::CONTINUOUS));

      aqts[id]->_vision_threads->add_waiting_thread(thread, raw);

    } else {
      Camera *cam = NULL;
      try {
	cam = CameraFactory::instance(cap);
	cam->open();
	cam->start();
      } catch (Exception &e) {
	delete cam;
	delete cap;
	e.append("Could not open or start camera");
	throw;
      }

      FvAquisitionThread *aqt = new FvAquisitionThread(id.c_str(), cam, logger, clock);

      c = aqt->camera_instance(raw, (vision_thread->vision_thread_mode() ==
				     VisionAspect::CONTINUOUS));

      aqt->_vision_threads->add_waiting_thread(thread, raw);

      aqts[id] = aqt;
      aqt->start();

      // no need to recreate barrier, by default aqts operate in continuous mode

      logger->log_info(name(), "Aquisition thread '%s' started for thread '%s' and camera '%s'",
		       aqt->name(), thread->name(), id.c_str());

    }

    thread->add_notification_listener(this);

  } catch (UnknownCameraTypeException &e) {
    delete cap;
    e.append("FvBaseVisionMaster: could not instantiate camera");
    aqts.unlock();
    throw;
  } catch (Exception &e) {
    delete cap;
    e.append("FvBaseVisionMaster: could not open or start camera");
    aqts.unlock();
    throw;
  }

  delete cap;

  aqts.unlock();
  return c;
}


/** Conditionally re-create barriers.
 * Re-create barriers if the number of cyclic threads has changed.
 * @param num_cyclic_threads new number of cyclic threads
 */
void
FvBaseThread::cond_recreate_barrier(unsigned int num_cyclic_threads)
{
  if ( (num_cyclic_threads + 1) != aqt_barrier->count() ) {
    delete aqt_barrier;
    aqt_barrier = new Barrier( num_cyclic_threads + 1 ); // +1 for base thread
  }
}


void
FvBaseThread::unregister_thread(Thread *thread)
{
  aqts.lock();
  unsigned int num_cyclic_threads = 0;

  for (ait = aqts.begin(); ait != aqts.end(); ++ait) {

    // Remove thread from all aqts
    (*ait).second->_vision_threads->remove_thread(thread);

    if ( (*ait).second->_vision_threads->has_cyclic_thread() ) {
      ++num_cyclic_threads;

    } else if ((*ait).second->aqtmode() != FvAquisitionThread::AqtContinuous ) {
      logger->log_info(name(), "Switching aquisition thread %s to continuous mode "
		               "on unregister", (*ait).second->name());
      (*ait).second->cancel();
      (*ait).second->join();
      (*ait).second->set_aqtmode(FvAquisitionThread::AqtContinuous);
      (*ait).second->start();
    }
  }
  // Recreate as necessary after _removing_ threads
  cond_recreate_barrier(num_cyclic_threads);

  aqts.unlock();
}


void
FvBaseThread::thread_started(Thread *thread)
{
  aqts.lock();
  //logger->log_debug(name(), "Thread %s started", thread->name());

  for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
    if ((*ait).second->_vision_threads->has_waiting_thread(thread)) {
      started_threads[thread] = (*ait).second;
    }
  }
  aqts.unlock();
}


void
FvBaseThread::thread_init_failed(Thread *thread)
{
  aqts.lock();
  for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
    (*ait).second->_vision_threads->remove_waiting_thread(thread);
  }
  aqts.unlock();
}
