
/***************************************************************************
 *  acquisition_thread.h - FireVision Acquisition Thread
 *
 *  Created: Wed Jun 06 19:01:10 2007
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

#include "acquisition_thread.h"
#include "aqt_vision_threads.h"

#include <core/exceptions/system.h>
#include <core/exceptions/software.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>
#ifdef FVBASE_TIMETRACKER
#include <utils/time/clock.h>
#include <utils/time/tracker.h>
#endif
#include <logging/logger.h>

#include <fvcams/shmem.h>
#include <fvutils/color/conversions.h>
#include <interfaces/SwitchInterface.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <cstdio>
#include <cstdlib>
#include <string>
#include <algorithm>

using namespace fawkes;
using namespace firevision;

/** @class FvAcquisitionThread "acquisition_thread.h"
 * FireVision base application acquisition thread.
 * This thread is used by the base application to acquire images from a camera
 * and call dependant threads when new images are available so that these
 * threads can start processing the images.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger logger
 * @param id id to be used for the shared memory segment and to announce changes
 * to the base thread
 * @param camera camera to manage
 * @param clock clock to use for timeout measurement (system time)
 */
FvAcquisitionThread::FvAcquisitionThread(const char *id,  Camera *camera,
					 Logger *logger, Clock *clock)
  : Thread("FvAcquisitionThread"),
    BlackBoardInterfaceListener("FvAcquisitionThread::%s", id)
{
  set_prepfin_conc_loop(true);
  set_name("FvAcquisitionThread::%s", id);

  __image_id      = strdup(id);

  vision_threads  = new FvAqtVisionThreads(clock);
  raw_subscriber_thread = NULL;

  __enabled_mutex = new Mutex(Mutex::RECURSIVE);
  __enabled_waitcond = new WaitCondition(__enabled_mutex);

  __camera        = camera;
  __width         = __camera->pixel_width();
  __height        = __camera->pixel_height();
  __colorspace    = __camera->colorspace();

  __mode = AqtContinuous;
  __enabled = false;

#ifdef FVBASE_TIMETRACKER
  __tt = new TimeTracker();
  __loop_count = 0;
  __ttc_capture = __tt->add_class("Capture");
  __ttc_lock    = __tt->add_class("Lock");
  __ttc_convert = __tt->add_class("Convert");
  __ttc_unlock  = __tt->add_class("Unlock");
  __ttc_dispose = __tt->add_class("Dispose");
#endif
}


/** Destructor. */
FvAcquisitionThread::~FvAcquisitionThread()
{
  __camera->close();

  for (__shmit = __shm.begin(); __shmit != __shm.end(); ++__shmit) {
    delete __shmit->second;
  }
  __shm.clear();

  delete vision_threads;
  delete __camera;
  free(__image_id);
  delete __enabled_waitcond;
  delete __enabled_mutex;
}

void
FvAcquisitionThread::init()
{
  logger->log_debug(name(), "Camera opened, w=%u  h=%u  c=%s", __width, __height,
		    colorspace_to_string(__colorspace));

  std::string if_id = std::string("Camera ") + __image_id;
  __enabled_if = blackboard->open_for_writing<SwitchInterface>(if_id.c_str());
  __enabled_if->set_enabled(__enabled);
  __enabled_if->write();

  bbil_add_message_interface(__enabled_if);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);
}


void
FvAcquisitionThread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->close(__enabled_if);
}


/** Get a camera instance.
 * This will return a camera instance suitable for accessing the image
 * buffer. Note, that this is not the camera provided to the constructor,
 * but rather a SharedMemoryCamera instance accessing a shared memory buffer
 * where the image is copied to (or a conversion result is posted to).
 * The returned instance has to bee freed using delete when done with it.
 *
 * You can decide whether you want to get access to the raw camera image
 * that has not been modified in any way or to the YUV422_PLANAR image buffer
 * (a conversion is done if needed). Use the raw parameter to decide whether
 * to get the raw image (true) or the YUV422_PLANAR image (false).
 *
 * When a thread is added it is internally put into a waiting queue. Since
 * at the time when it is added the thread is not yet started, and its
 * initialization may even fail. For this reason the acquisition thread
 * registers itself to receive status notifications of the thread. If the
 * thread signals successful startup it is moved to the running queue and
 * from then on woken up when new image material can be processed. If the
 * thread fails for whatever reason it is dropped.
 *
 * The acquisition thread has a timeout. If no thread is in the running or
 * waiting queue for this number of seconds, the base thread is signalled
 * to shut down this acquisition thread (which the base thread may do or
 * deny). This is done so that if a plugin is just unloaded shortly and
 * then quickly loaded again the overhead of closing the camera and then
 * opening it again is avoided.
 *
 * @param cspace the desired colorspace the image should be converted to.
 * See general notes in VisionMaster::register_for_camera().
 * @param deep_copy given to the shared memory camera.
 * @return camera instance
 * @see SharedMemoryCamera
 */
Camera *
FvAcquisitionThread::camera_instance(colorspace_t cspace, bool deep_copy)
{
  const char *img_id = NULL;

  if (cspace == CS_UNKNOWN) {
    if (raw_subscriber_thread) {
      // There may be only one
      throw Exception("Only one vision thread may access the raw camera.");
    } else {
      return __camera;
    }
  } else {
    char *tmp =  NULL;
    if (__shm.find(cspace) == __shm.end()) {
      if ( asprintf(&tmp, "%s.%zu", __image_id, __shm.size()) == -1) {
	throw OutOfMemoryException("FvAcqThread::camera_instance(): Could not create image ID");
      }
      img_id = tmp;
      __shm[cspace] = new SharedMemoryImageBuffer(img_id, cspace, __width, __height);
    } else {
      img_id = __shm[cspace]->image_id();
    }

    SharedMemoryCamera *c = new SharedMemoryCamera(img_id, deep_copy);

    if (tmp)  free(tmp);
    return c;
  }
}


/** Get the Camera of this acquisition thread.
 * This is just used for the camera controls, if you want to access the camera,
 * use camera_instance()
 * @return a pointer to the Camera
 */
Camera *
FvAcquisitionThread::get_camera()
{
  return __camera;
}


/** Set acquisition thread mode.
 * Note that this may only be called on a stopped thread or an
 * exception will be thrown by Thread::set_opmode()!
 * @param mode new acquisition thread mode
 */
void
FvAcquisitionThread::set_aqtmode(AqtMode mode)
{
  if ( mode == AqtCyclic ) {
    //logger->log_info(name(), "Setting WAITFORWAKEUPMODE");
    set_opmode(Thread::OPMODE_WAITFORWAKEUP);
  } else if ( mode == AqtContinuous ) {
    //logger->log_info(name(), "Setting CONTINUOUS");
    set_opmode(Thread::OPMODE_CONTINUOUS);
  }
  __mode = mode;
}


/** Enable or disable image retrieval.
 * When the acquisition thread is enabled image data will be converted or copied
 * to the shared memory buffer, otherwise only the capture/dispose cycle is
 * executed.
 * @param enabled true to enable acquisition thread, false to disable
 */
void
FvAcquisitionThread::set_enabled(bool enabled)
{
  MutexLocker lock(__enabled_mutex);

  if (__enabled && ! enabled) {
    // disabling thread
    __camera->stop();
    __enabled_if->set_enabled(false);
    __enabled_if->write();

  } else if (! __enabled && enabled) {
    // enabling thread
    __camera->start();
    __enabled_if->set_enabled(true);
    __enabled_if->write();

    __enabled_waitcond->wake_all();
  } // else not state change

  // we can safely do this every time...
  __enabled = enabled;
}


/** Get acquisition thread mode.
 * @return acquisition thread mode.
 */
FvAcquisitionThread::AqtMode
FvAcquisitionThread::aqtmode()
{
  return __mode;
}


/** Set prepfin hold status for vision threads.
 * @param hold prepfin hold status
 * @see Thread::set_prepfin_hold()
 */
void
FvAcquisitionThread::set_vt_prepfin_hold(bool hold)
{
  try {
    vision_threads->set_prepfin_hold(hold);
  } catch (Exception &e) {
    logger->log_warn(name(), "At least one thread was being finalized while prepfin hold "
		      "was about to be acquired");
    throw;
  }
}


void
FvAcquisitionThread::loop()
{
  MutexLocker lock(__enabled_mutex);

  while (! __enabled_if->msgq_empty() ) {
    if (__enabled_if->msgq_first_is<SwitchInterface::EnableSwitchMessage>()) {
      // must be re-established
      logger->log_info(name(), "Enabling on blackboard request");
      set_enabled(true);
    } else if (__enabled_if->msgq_first_is<SwitchInterface::DisableSwitchMessage>()) {
      logger->log_info(name(), "Disabling on blackboard request");
      set_enabled(false);
    } else {
      logger->log_warn(name(), "Unhandled message %s ignored",
		       __enabled_if->msgq_first()->type());
    }
    __enabled_if->msgq_pop();
  }

  // We disable cancelling here to avoid problems with the write lock
  Thread::CancelState old_cancel_state;
  set_cancel_state(Thread::CANCEL_DISABLED, &old_cancel_state);

#ifdef FVBASE_TIMETRACKER
  try {
    if ( __enabled ) {
      __tt->ping_start(__ttc_capture);
      __camera->capture();
      __tt->ping_end(__ttc_capture);

      for (__shmit = __shm.begin(); __shmit != __shm.end(); ++__shmit) {
	if (__shmit->first == CS_UNKNOWN)  continue;
	__tt->ping_start(__ttc_lock);
	__shmit->second->lock_for_write();
	__tt->ping_end(__ttc_lock);
	__tt->ping_start(__ttc_convert);
	convert(__colorspace, __shmit->first,
		__camera->buffer(), __shmit->second->buffer(),
		__width, __height);
	try {
	  __shmit->second->set_capture_time(__camera->capture_time());
	} catch (NotImplementedException &e) {
	  // ignored
	}
	__tt->ping_end(__ttc_convert);
	__tt->ping_start(__ttc_unlock);
	__shmit->second->unlock();
	__tt->ping_end(__ttc_unlock);
      }
    }
  } catch (Exception &e) {
    logger->log_error(name(), "Cannot convert image data");
    logger->log_error(name(), e);
  }
  if ( __enabled ) {
    __tt->ping_start(__ttc_dispose);
    __camera->dispose_buffer();
    __tt->ping_end(__ttc_dispose);
  }

  if ( (++__loop_count % FVBASE_TT_PRINT_INT) == 0 ) {
    __tt->print_to_stdout();
  }

#else // no time tracking
  try {
    if ( __enabled ) {
      __camera->capture();
      for (__shmit = __shm.begin(); __shmit != __shm.end(); ++__shmit) {
	if (__shmit->first == CS_UNKNOWN)  continue;
	__shmit->second->lock_for_write();
	convert(__colorspace, __shmit->first,
		__camera->buffer(), __shmit->second->buffer(),
		__width, __height);
	try {
	  __shmit->second->set_capture_time(__camera->capture_time());
	} catch (NotImplementedException &e) {
	  // ignored
	}
	__shmit->second->unlock();
      }
    }
  } catch (Exception &e) {
    logger->log_error(name(), e);
  }
  if ( __enabled ) {
    __camera->dispose_buffer();
  }
#endif

  if ( __mode == AqtCyclic && __enabled ) {
    vision_threads->wakeup_and_wait_cyclic_threads();
  }

  // reset to the original cancel state, cancelling is now safe
  set_cancel_state(old_cancel_state);

  // in continuous mode wait for signal if disabled
  while ( __mode == AqtContinuous && ! __enabled ) {
    __enabled_waitcond->wait();
  }
}

bool
FvAcquisitionThread::bb_interface_message_received(Interface *interface,
						   Message *message) throw()
{
  // in continuous mode wait for signal if disabled
  MutexLocker lock(__enabled_mutex);
  if (__mode == AqtContinuous && ! __enabled) {
    if (message->is_of_type<SwitchInterface::EnableSwitchMessage>()) {
      logger->log_info(name(), "Enabling on blackboard request");
      set_enabled(true);
      return false;
    }
  }

  return true;
}
