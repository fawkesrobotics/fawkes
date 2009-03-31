
/***************************************************************************
 *  base_thread.cpp - FireVision Base Thread
 *
 *  Created: Tue May 29 16:41:50 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <apps/base/base_thread.h>
#include <apps/base/acquisition_thread.h>
#include <apps/base/aqt_vision_threads.h>

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/threading/barrier.h>
#include <utils/logging/logger.h>

#include <fvutils/system/camargp.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/ipc/shm_lut.h>
#include <cams/factory.h>

#include <aspect/vision.h>

#include <unistd.h>

using namespace fawkes;

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
  // wipe all previously existing FireVision shared memory segments
  // that are orphaned
  SharedMemoryImageBuffer::cleanup(/* use lister */ false);
  SharedMemoryLookupTable::cleanup(/* use lister */ false);
}


void
FvBaseThread::finalize()
{
  aqts.lock();
  for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
    thread_collector->remove(ait->second);
    delete ait->second;
  }
  aqts.clear();
  aqts.unlock();
}


/** Thread loop. */
void
FvBaseThread::loop()
{
  aqts.lock();

  try {
    for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
      ait->second->set_vt_prepfin_hold(true);
    }
  } catch (Exception &e) {
    logger->log_warn(name(), "Cannot get prepfin hold status, skipping this loop");
    for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
      ait->second->set_vt_prepfin_hold(false);
    }
    aqts.unlock();
    return;
  }

  // Wakeup all cyclic acquisition threads and wait for them
  for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
    if ( ait->second->aqtmode() == FvAcquisitionThread::AqtCyclic ) {
      //logger->log_debug(name(), "Waking Thread %s", ait->second->name());
      ait->second->wakeup(aqt_barrier);
    }
  }

  aqt_barrier->wait();

  // Check for aqt timeouts
  for (ait = aqts.begin(); ait != aqts.end();) {
    if ( ait->second->_vision_threads->empty() &&
	 (ait->second->_vision_threads->empty_time() > _aqt_timeout) ) {

      logger->log_info(name(), "Acquisition thread %s timed out, destroying",
		       ait->second->name());


      thread_collector->remove(ait->second);
      delete ait->second;
      aqts.erase(ait++);
    } else {
      ++ait;
    }
  }

  started_threads.lock();
  fawkes::LockMap<Thread *, FvAcquisitionThread *>::iterator stit = started_threads.begin();
  while (stit != started_threads.end()) {

    logger->log_info(name(), "Thread %s has been started, %zu",
		     stit->second->name(), started_threads.size());

    // if the thread is registered in that aqt mark it running
    stit->second->_vision_threads->set_thread_running(stit->first);

    if ( stit->second->_vision_threads->has_cyclic_thread() ) {
      if (stit->second->aqtmode() != FvAcquisitionThread::AqtCyclic ) {
	logger->log_info(name(), "Switching acquisition thread %s to cyclic mode",
			 stit->second->name());

	stit->second->prepare_finalize();
	stit->second->cancel();
	stit->second->join();
	stit->second->set_aqtmode(FvAcquisitionThread::AqtCyclic);
	stit->second->start();
	stit->second->cancel_finalize();
      }
    } else if (stit->second->aqtmode() != FvAcquisitionThread::AqtContinuous ) {
      logger->log_info(name(), "Switching acquisition thread %s to continuous mode",
		       stit->second->name());
      stit->second->prepare_finalize();
      stit->second->cancel();
      stit->second->join();
      stit->second->set_aqtmode(FvAcquisitionThread::AqtContinuous);
      stit->second->start();
      stit->second->cancel_finalize();
    }

    // Make thread actually capture data
    stit->second->set_enabled(true);

    fawkes::LockMap<Thread *, FvAcquisitionThread *>::iterator stittmp = stit;
    ++stit;
    started_threads.erase( stittmp );
  }
  started_threads.unlock();

  // Re-create barrier as necessary after _adding_ threads
  unsigned int num_cyclic_threads = 0;
  for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
    if ( ait->second->_vision_threads->has_cyclic_thread() ) {
      ++num_cyclic_threads;
    }
  }
  cond_recreate_barrier(num_cyclic_threads);

  for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
    ait->second->set_vt_prepfin_hold(false);
  }

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
    std::string id = cap->cam_type() + "." + cap->cam_id();
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
	e.append("Could not open or start camera");
	throw;
      }

      FvAcquisitionThread *aqt = new FvAcquisitionThread(id.c_str(), cam, logger, clock);

      c = aqt->camera_instance(raw, (vision_thread->vision_thread_mode() ==
				     VisionAspect::CONTINUOUS));

      aqt->_vision_threads->add_waiting_thread(thread, raw);

      aqts[id] = aqt;
      thread_collector->add(aqt);

      // no need to recreate barrier, by default aqts operate in continuous mode

      logger->log_info(name(), "Acquisition thread '%s' started for thread '%s' and camera '%s'",
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
    ait->second->_vision_threads->remove_thread(thread);

    if ( ait->second->_vision_threads->has_cyclic_thread() ) {
      ++num_cyclic_threads;

    } else if (ait->second->aqtmode() != FvAcquisitionThread::AqtContinuous ) {
      logger->log_info(name(), "Switching acquisition thread %s to continuous mode "
		               "on unregister", ait->second->name());

      ait->second->prepare_finalize();
      ait->second->cancel();
      ait->second->join();
      ait->second->set_aqtmode(FvAcquisitionThread::AqtContinuous);
      ait->second->start();
      ait->second->cancel_finalize();
    }
  }
  // Recreate as necessary after _removing_ threads
  cond_recreate_barrier(num_cyclic_threads);

  aqts.unlock();
}


bool
FvBaseThread::thread_started(Thread *thread) throw()
{
  aqts.lock();
  for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
    if (ait->second->_vision_threads->has_waiting_thread(thread)) {
      started_threads.lock();
      started_threads[thread] = ait->second;
      started_threads.unlock();
    }
  }
  aqts.unlock();

  return false;
}


bool
FvBaseThread::thread_init_failed(Thread *thread) throw()
{
  aqts.lock();
  for (ait = aqts.begin(); ait != aqts.end(); ++ait) {
    ait->second->_vision_threads->remove_waiting_thread(thread);
  }
  aqts.unlock();

  return false;
}
