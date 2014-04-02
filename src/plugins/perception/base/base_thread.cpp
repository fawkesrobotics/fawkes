
/***************************************************************************
 *  base_thread.cpp - FireVision Base Thread
 *
 *  Created: Tue May 29 16:41:50 2007
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include "base_thread.h"
#include "acquisition_thread.h"
#include "aqt_vision_threads.h"

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/barrier.h>
#include <logging/logger.h>

#include <fvutils/system/camargp.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/ipc/shm_lut.h>
#include <fvcams/factory.h>
#include <fvcams/cam_exceptions.h>
#include <fvcams/control/factory.h>
#include <core/exceptions/software.h>

#include <aspect/vision.h>

#include <algorithm>
#include <unistd.h>

using namespace fawkes;
using namespace firevision;

/** @class FvBaseThread "base_thread.h"
 * FireVision base thread.
 * This implements the functionality of the FvBasePlugin.
 * @author Tim Niemueller
 */

/** Constructor. */
FvBaseThread::FvBaseThread()
  : Thread("FvBaseThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
    VisionMasterAspect(this)
{
  // default to 30 seconds
  __aqt_timeout = 30;
  __aqt_barrier = new Barrier(1);
}


/** Destructor. */
FvBaseThread::~FvBaseThread()
{
  delete __aqt_barrier;
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
  __aqts.lock();
  for (__ait = __aqts.begin(); __ait != __aqts.end(); ++__ait) {
    thread_collector->remove(__ait->second);
    delete __ait->second;
  }
  __aqts.clear();
  __aqts.unlock();
  __owned_controls.lock();
  LockList<CameraControl *>::iterator i;
  for (i = __owned_controls.begin(); i != __owned_controls.end(); ++i) {
    delete *i;
  }
  __owned_controls.clear();
  __owned_controls.unlock();
}


/** Thread loop. */
void
FvBaseThread::loop()
{
  __aqts.lock();

  try {
    for (__ait = __aqts.begin(); __ait != __aqts.end(); ++__ait) {
      __ait->second->set_vt_prepfin_hold(true);
    }
  } catch (Exception &e) {
    logger->log_warn(name(), "Cannot get prepfin hold status, skipping this loop");
    for (__ait = __aqts.begin(); __ait != __aqts.end(); ++__ait) {
      __ait->second->set_vt_prepfin_hold(false);
    }
    __aqts.unlock();
    return;
  }

  // Wakeup all cyclic acquisition threads and wait for them
  for (__ait = __aqts.begin(); __ait != __aqts.end(); ++__ait) {
    if ( __ait->second->aqtmode() == FvAcquisitionThread::AqtCyclic ) {
      //logger->log_debug(name(), "Waking Thread %s", __ait->second->name());
      __ait->second->wakeup(__aqt_barrier);
    }
  }

  __aqt_barrier->wait();

  // Check for aqt timeouts
  for (__ait = __aqts.begin(); __ait != __aqts.end();) {
    if ( __ait->second->vision_threads->empty() &&
	 (__ait->second->vision_threads->empty_time() > __aqt_timeout) ) {

      logger->log_info(name(), "Acquisition thread %s timed out, destroying",
		       __ait->second->name());


      thread_collector->remove(__ait->second);
      delete __ait->second;
      __aqts.erase(__ait++);
    } else {
      ++__ait;
    }
  }

  __started_threads.lock();
  fawkes::LockMap<Thread *, FvAcquisitionThread *>::iterator stit = __started_threads.begin();
  while (stit != __started_threads.end()) {

    logger->log_info(name(), "Thread %s has been started, %zu",
		     stit->second->name(), __started_threads.size());

    // if the thread is registered in that aqt mark it running
    stit->second->vision_threads->set_thread_running(stit->first);

    if ( stit->second->vision_threads->has_cyclic_thread() ) {
      // Make thread actually capture data
      stit->second->set_enabled(true);

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
    } else if ( stit->second->vision_threads->has_cont_thread() ) {
      // Make thread actually capture data
      stit->second->set_enabled(true);

      if (stit->second->aqtmode() != FvAcquisitionThread::AqtContinuous ) {
	logger->log_info(name(), "Switching acquisition thread %s to continuous mode",
			 stit->second->name());
	stit->second->prepare_finalize();
	stit->second->cancel();
	stit->second->join();
	stit->second->set_aqtmode(FvAcquisitionThread::AqtContinuous);
	stit->second->start();
	stit->second->cancel_finalize();
      }
    } else {
      logger->log_warn(name(), "Acquisition thread %s has no threads while we expected some",
		       stit->second->name());
      // Make thread stop capturing data
      stit->second->set_enabled(false);
    }

    fawkes::LockMap<Thread *, FvAcquisitionThread *>::iterator stittmp = stit;
    ++stit;
    __started_threads.erase( stittmp );
  }
  __started_threads.unlock();

  // Re-create barrier as necessary after _adding_ threads
  unsigned int num_cyclic_threads = 0;
  for (__ait = __aqts.begin(); __ait != __aqts.end(); ++__ait) {
    if ( __ait->second->vision_threads->has_cyclic_thread() ) {
      ++num_cyclic_threads;
    }
  }
  cond_recreate_barrier(num_cyclic_threads);

  for (__ait = __aqts.begin(); __ait != __aqts.end(); ++__ait) {
    __ait->second->set_vt_prepfin_hold(false);
  }

  __aqts.unlock();
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
FvBaseThread::register_for_camera(const char *camera_string, Thread *thread,
				  colorspace_t cspace)
{
  Camera *c = NULL;
  __aqts.lock();

  logger->log_info(name(), "Thread '%s' registers for camera '%s'", thread->name(), camera_string);

  VisionAspect *vision_thread = dynamic_cast<VisionAspect *>(thread);
  if ( vision_thread == NULL ) {
    throw TypeMismatchException("Thread is not a vision thread");
  }

  CameraArgumentParser *cap = new CameraArgumentParser(camera_string);
  try {
    std::string id = cap->cam_type() + "." + cap->cam_id();
    if ( __aqts.find(id) != __aqts.end() ) {
      // this camera has already been loaded
      c = __aqts[id]->camera_instance(cspace,
				    (vision_thread->vision_thread_mode() ==
				     VisionAspect::CONTINUOUS));

      __aqts[id]->vision_threads->add_waiting_thread(thread);

    } else {
      Camera *cam = NULL;
      try {
	cam = CameraFactory::instance(cap);
	cam->open();
      } catch (Exception &e) {
	delete cam;
	e.append("Could not open camera");
	throw;
      }

      FvAcquisitionThread *aqt = new FvAcquisitionThread(id.c_str(), cam, logger, clock);

      c = aqt->camera_instance(cspace, (vision_thread->vision_thread_mode() ==
					VisionAspect::CONTINUOUS));

      aqt->vision_threads->add_waiting_thread(thread);

      __aqts[id] = aqt;
      thread_collector->add(aqt);

      // no need to recreate barrier, by default aqts operate in continuous mode

      logger->log_info(name(), "Acquisition thread '%s' started for thread '%s' and camera '%s'",
		       aqt->name(), thread->name(), id.c_str());

    }

    thread->add_notification_listener(this);

  } catch (UnknownCameraTypeException &e) {
    delete cap;
    e.append("FvBaseVisionMaster: could not instantiate camera");
    __aqts.unlock();
    throw;
  } catch (Exception &e) {
    delete cap;
    e.append("FvBaseVisionMaster: could not open or start camera");
    __aqts.unlock();
    throw;
  }

  delete cap;

  __aqts.unlock();
  return c;
}


Camera *
FvBaseThread::register_for_raw_camera(const char *camera_string, Thread *thread)
{
  Camera *camera = register_for_camera(camera_string, thread, CS_UNKNOWN);
  CameraArgumentParser cap(camera_string);
  try {
    std::string id = cap.cam_type() + "." + cap.cam_id();
    __aqts.lock();
    if ( __aqts.find(id) != __aqts.end() ) {
      __aqts[id]->raw_subscriber_thread = thread;
    }
    __aqts.unlock();
  } catch (Exception &e) {
    __aqts.unlock();
    throw;
  }
  return camera;
}

CameraControl *
FvBaseThread::create_camctrl(const char *camera_string)
{
  CameraControl *cc = CameraControlFactory::instance(camera_string);
  if (cc) {
    __owned_controls.lock();
    __owned_controls.push_back(cc);
    __owned_controls.sort();
    __owned_controls.unique();
    __owned_controls.unlock();
    return cc;
  } else {
    throw Exception("Cannot create camera control of desired type");
  }
}

CameraControl *
FvBaseThread::acquire_camctrl(const char *cam_string)
{
  CameraArgumentParser cap(cam_string);
  std::string id = cap.cam_type() + "." + cap.cam_id();

  // Has this camera been loaded?
  MutexLocker lock(__aqts.mutex());
  if (__aqts.find(id) != __aqts.end()) {
    return CameraControlFactory::instance(__aqts[id]->get_camera());
  } else {
    return create_camctrl(cam_string);
  }
}


CameraControl *
FvBaseThread::acquire_camctrl(const char *cam_string,
			      const std::type_info &typeinf)
{
  CameraArgumentParser cap(cam_string);
  std::string id = cap.cam_type() + "." + cap.cam_id();

  // Has this camera been loaded?
  MutexLocker lock(__aqts.mutex());
  if (__aqts.find(id) != __aqts.end()) {
    return CameraControlFactory::instance(typeinf, __aqts[id]->get_camera());
  } else {
    return create_camctrl(cam_string);
  }
}


void
FvBaseThread::release_camctrl(CameraControl *cc)
{
  __owned_controls.lock();
  LockList<CameraControl *>::iterator f;
  if ((f = std::find(__owned_controls.begin(), __owned_controls.end(), cc)) != __owned_controls.end()) {
    delete *f;
    __owned_controls.erase(f);
  }
  __owned_controls.unlock();
}


/** Conditionally re-create barriers.
 * Re-create barriers if the number of cyclic threads has changed.
 * @param num_cyclic_threads new number of cyclic threads
 */
void
FvBaseThread::cond_recreate_barrier(unsigned int num_cyclic_threads)
{
  if ( (num_cyclic_threads + 1) != __aqt_barrier->count() ) {
    delete __aqt_barrier;
    __aqt_barrier = new Barrier( num_cyclic_threads + 1 ); // +1 for base thread
  }
}


void
FvBaseThread::unregister_thread(Thread *thread)
{
  __aqts.lock();
  unsigned int num_cyclic_threads = 0;

  for (__ait = __aqts.begin(); __ait != __aqts.end(); ++__ait) {

    // Remove thread from all aqts
    __ait->second->vision_threads->remove_thread(thread);

    if (__ait->second->raw_subscriber_thread == thread) {
      __ait->second->raw_subscriber_thread = NULL;
    }

    if ( __ait->second->vision_threads->has_cyclic_thread() ) {
      ++num_cyclic_threads;

    } else if (__ait->second->aqtmode() != FvAcquisitionThread::AqtContinuous ) {
      logger->log_info(name(), "Switching acquisition thread %s to continuous mode "
		               "on unregister", __ait->second->name());

      __ait->second->prepare_finalize();
      __ait->second->cancel();
      __ait->second->join();
      __ait->second->set_aqtmode(FvAcquisitionThread::AqtContinuous);
      __ait->second->start();
      __ait->second->cancel_finalize();
    }

    if (__ait->second->vision_threads->empty()) {
      // Make thread stop capturing data
      logger->log_info(name(), "Disabling capturing on thread %s (no more threads)",
		       __ait->second->name());
      __ait->second->set_enabled(false);
    }
  }
  // Recreate as necessary after _removing_ threads
  cond_recreate_barrier(num_cyclic_threads);

  __aqts.unlock();
}


bool
FvBaseThread::thread_started(Thread *thread) throw()
{
  __aqts.lock();
  for (__ait = __aqts.begin(); __ait != __aqts.end(); ++__ait) {
    if (__ait->second->vision_threads->has_waiting_thread(thread)) {
      __started_threads.lock();
      __started_threads[thread] = __ait->second;
      __started_threads.unlock();
    }
  }
  __aqts.unlock();

  return false;
}


bool
FvBaseThread::thread_init_failed(Thread *thread) throw()
{
  __aqts.lock();
  for (__ait = __aqts.begin(); __ait != __aqts.end(); ++__ait) {
    __ait->second->vision_threads->remove_waiting_thread(thread);
  }
  __aqts.unlock();

  return false;
}
