
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
  aqt_timeout_ = 30;
  aqt_barrier_ = new Barrier(1);
}


/** Destructor. */
FvBaseThread::~FvBaseThread()
{
  delete aqt_barrier_;
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
  aqts_.lock();
  for (ait_ = aqts_.begin(); ait_ != aqts_.end(); ++ait_) {
    thread_collector->remove(ait_->second);
    delete ait_->second;
  }
  aqts_.clear();
  aqts_.unlock();
  owned_controls_.lock();
  LockList<CameraControl *>::iterator i;
  for (i = owned_controls_.begin(); i != owned_controls_.end(); ++i) {
    delete *i;
  }
  owned_controls_.clear();
  owned_controls_.unlock();
}


/** Thread loop. */
void
FvBaseThread::loop()
{
  aqts_.lock();

  try {
    for (ait_ = aqts_.begin(); ait_ != aqts_.end(); ++ait_) {
      ait_->second->set_vt_prepfin_hold(true);
    }
  } catch (Exception &e) {
    logger->log_warn(name(), "Cannot get prepfin hold status, skipping this loop");
    for (ait_ = aqts_.begin(); ait_ != aqts_.end(); ++ait_) {
      ait_->second->set_vt_prepfin_hold(false);
    }
    aqts_.unlock();
    return;
  }

  // Wakeup all cyclic acquisition threads and wait for them
  for (ait_ = aqts_.begin(); ait_ != aqts_.end(); ++ait_) {
    if ( ait_->second->aqtmode() == FvAcquisitionThread::AqtCyclic ) {
      //logger->log_debug(name(), "Waking Thread %s", ait_->second->name());
      ait_->second->wakeup(aqt_barrier_);
    }
  }

  aqt_barrier_->wait();

  // Check for aqt timeouts
  for (ait_ = aqts_.begin(); ait_ != aqts_.end();) {
    if ( ait_->second->vision_threads->empty() &&
	 (ait_->second->vision_threads->empty_time() > aqt_timeout_) ) {

      logger->log_info(name(), "Acquisition thread %s timed out, destroying",
		       ait_->second->name());


      thread_collector->remove(ait_->second);
      delete ait_->second;
      aqts_.erase(ait_++);
    } else {
      ++ait_;
    }
  }

  started_threads_.lock();
  fawkes::LockMap<Thread *, FvAcquisitionThread *>::iterator stit = started_threads_.begin();
  while (stit != started_threads_.end()) {

    logger->log_info(name(), "Thread %s has been started, %zu",
		     stit->second->name(), started_threads_.size());

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
    started_threads_.erase( stittmp );
  }
  started_threads_.unlock();

  // Re-create barrier as necessary after _adding_ threads
  unsigned int num_cyclic_threads = 0;
  for (ait_ = aqts_.begin(); ait_ != aqts_.end(); ++ait_) {
    if ( ait_->second->vision_threads->has_cyclic_thread() ) {
      ++num_cyclic_threads;
    }
  }
  cond_recreate_barrier(num_cyclic_threads);

  for (ait_ = aqts_.begin(); ait_ != aqts_.end(); ++ait_) {
    ait_->second->set_vt_prepfin_hold(false);
  }

  aqts_.unlock();
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
  aqts_.lock();

  logger->log_info(name(), "Thread '%s' registers for camera '%s'", thread->name(), camera_string);

  VisionAspect *vision_thread = dynamic_cast<VisionAspect *>(thread);
  if ( vision_thread == NULL ) {
    throw TypeMismatchException("Thread is not a vision thread");
  }

  CameraArgumentParser *cap = new CameraArgumentParser(camera_string);
  try {
    std::string id = cap->cam_type() + "." + cap->cam_id();
    if ( aqts_.find(id) != aqts_.end() ) {
      // this camera has already been loaded
      c = aqts_[id]->camera_instance(cspace,
				    (vision_thread->vision_thread_mode() ==
				     VisionAspect::CONTINUOUS));

      aqts_[id]->vision_threads->add_waiting_thread(thread);

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

      aqts_[id] = aqt;
      thread_collector->add(aqt);

      // no need to recreate barrier, by default aqts operate in continuous mode

      logger->log_info(name(), "Acquisition thread '%s' started for thread '%s' and camera '%s'",
		       aqt->name(), thread->name(), id.c_str());

    }

    thread->add_notification_listener(this);

  } catch (UnknownCameraTypeException &e) {
    delete cap;
    e.append("FvBaseVisionMaster: could not instantiate camera");
    aqts_.unlock();
    throw;
  } catch (Exception &e) {
    delete cap;
    e.append("FvBaseVisionMaster: could not open or start camera");
    aqts_.unlock();
    throw;
  }

  delete cap;

  aqts_.unlock();
  return c;
}


Camera *
FvBaseThread::register_for_raw_camera(const char *camera_string, Thread *thread)
{
  Camera *camera = register_for_camera(camera_string, thread, CS_UNKNOWN);
  CameraArgumentParser cap(camera_string);
  try {
    std::string id = cap.cam_type() + "." + cap.cam_id();
    aqts_.lock();
    if ( aqts_.find(id) != aqts_.end() ) {
      aqts_[id]->raw_subscriber_thread = thread;
    }
    aqts_.unlock();
  } catch (Exception &e) {
    aqts_.unlock();
    throw;
  }
  return camera;
}

CameraControl *
FvBaseThread::create_camctrl(const char *camera_string)
{
  CameraControl *cc = CameraControlFactory::instance(camera_string);
  if (cc) {
    owned_controls_.lock();
    owned_controls_.push_back(cc);
    owned_controls_.sort();
    owned_controls_.unique();
    owned_controls_.unlock();
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
  MutexLocker lock(aqts_.mutex());
  if (aqts_.find(id) != aqts_.end()) {
    return CameraControlFactory::instance(aqts_[id]->get_camera());
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
  MutexLocker lock(aqts_.mutex());
  if (aqts_.find(id) != aqts_.end()) {
    return CameraControlFactory::instance(typeinf, aqts_[id]->get_camera());
  } else {
    return create_camctrl(cam_string);
  }
}


void
FvBaseThread::release_camctrl(CameraControl *cc)
{
  owned_controls_.lock();
  LockList<CameraControl *>::iterator f;
  if ((f = std::find(owned_controls_.begin(), owned_controls_.end(), cc)) != owned_controls_.end()) {
    delete *f;
    owned_controls_.erase(f);
  }
  owned_controls_.unlock();
}


/** Conditionally re-create barriers.
 * Re-create barriers if the number of cyclic threads has changed.
 * @param num_cyclic_threads new number of cyclic threads
 */
void
FvBaseThread::cond_recreate_barrier(unsigned int num_cyclic_threads)
{
  if ( (num_cyclic_threads + 1) != aqt_barrier_->count() ) {
    delete aqt_barrier_;
    aqt_barrier_ = new Barrier( num_cyclic_threads + 1 ); // +1 for base thread
  }
}


void
FvBaseThread::unregister_thread(Thread *thread)
{
  aqts_.lock();
  unsigned int num_cyclic_threads = 0;

  for (ait_ = aqts_.begin(); ait_ != aqts_.end(); ++ait_) {

    // Remove thread from all aqts
    ait_->second->vision_threads->remove_thread(thread);

    if (ait_->second->raw_subscriber_thread == thread) {
      ait_->second->raw_subscriber_thread = NULL;
    }

    if ( ait_->second->vision_threads->has_cyclic_thread() ) {
      ++num_cyclic_threads;

    } else if (ait_->second->aqtmode() != FvAcquisitionThread::AqtContinuous ) {
      logger->log_info(name(), "Switching acquisition thread %s to continuous mode "
		               "on unregister", ait_->second->name());

      ait_->second->prepare_finalize();
      ait_->second->cancel();
      ait_->second->join();
      ait_->second->set_aqtmode(FvAcquisitionThread::AqtContinuous);
      ait_->second->start();
      ait_->second->cancel_finalize();
    }

    if (ait_->second->vision_threads->empty()) {
      // Make thread stop capturing data
      logger->log_info(name(), "Disabling capturing on thread %s (no more threads)",
		       ait_->second->name());
      ait_->second->set_enabled(false);
    }
  }
  // Recreate as necessary after _removing_ threads
  cond_recreate_barrier(num_cyclic_threads);

  aqts_.unlock();
}


bool
FvBaseThread::thread_started(Thread *thread) throw()
{
  aqts_.lock();
  for (ait_ = aqts_.begin(); ait_ != aqts_.end(); ++ait_) {
    if (ait_->second->vision_threads->has_waiting_thread(thread)) {
      started_threads_.lock();
      started_threads_[thread] = ait_->second;
      started_threads_.unlock();
    }
  }
  aqts_.unlock();

  return false;
}


bool
FvBaseThread::thread_init_failed(Thread *thread) throw()
{
  aqts_.lock();
  for (ait_ = aqts_.begin(); ait_ != aqts_.end(); ++ait_) {
    ait_->second->vision_threads->remove_waiting_thread(thread);
  }
  aqts_.unlock();

  return false;
}
