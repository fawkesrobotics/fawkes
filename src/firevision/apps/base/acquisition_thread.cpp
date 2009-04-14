
/***************************************************************************
 *  acquisition_thread.h - FireVision Acquisition Thread
 *
 *  Created: Wed Jun 06 19:01:10 2007
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include "acquisition_thread.h"
#include "aqt_vision_threads.h"

#include <core/exceptions/system.h>
#include <core/exceptions/software.h>
#ifdef FVBASE_TIMETRACKER
#include <utils/time/clock.h>
#include <utils/time/tracker.h>
#endif
#include <utils/logging/logger.h>

#include <cams/shmem.h>
#include <fvutils/color/conversions.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <cstdio>
#include <string>
#include <algorithm>

using namespace fawkes;

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
  : Thread((std::string("FvAcquisitionThread::") + id).c_str())
{
  __logger        = logger;
  __image_id      = strdup(id);

  vision_threads  = new FvAqtVisionThreads(clock);

  __camera        = camera;
  __width         = __camera->pixel_width();
  __height        = __camera->pixel_height();
  __colorspace    = __camera->colorspace();
  logger->log_debug(name(), "Camera opened, w=%u  h=%u  c=%s", __width, __height,
		    colorspace_to_string(__colorspace));

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

  delete vision_threads;
  delete __camera;
  free(__image_id);
}


/** Get a camera instace.
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
SharedMemoryCamera *
FvAcquisitionThread::camera_instance(colorspace_t cspace, bool deep_copy)
{
  const char *img_id = NULL;
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


/** Set acquisition thread mode.
 * Note that this may only be called on a stopped thread or an
 * exception will be thrown by Thread::set_opmode()!
 * @param mode new acquisition thread mode
 */
void
FvAcquisitionThread::set_aqtmode(AqtMode mode)
{
  if ( mode == AqtCyclic ) {
    //__logger->log_info(name(), "Setting WAITFORWAKEUPMODE");
    set_opmode(Thread::OPMODE_WAITFORWAKEUP);
  } else if ( mode == AqtContinuous ) {
    //__logger->log_info(name(), "Setting CONTINUOUS");
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
    __logger->log_warn(name(), "At least one thread was being finalized while prepfin hold "
		      "was about to be acquired");
    throw;
  }
}


void
FvAcquisitionThread::loop()
{
  // We disable cancelling here to avoid problems with the write lock
  Thread::CancelState old_cancel_state;
  set_cancel_state(Thread::CANCEL_DISABLED, &old_cancel_state);

#ifdef FVBASE_TIMETRACKER
  try {
    __tt->ping_start(__ttc_capture);
    __camera->capture();
    __tt->ping_end(__ttc_capture);

    if ( __enabled ) {
      for (__shmit = __shm.begin(); __shmit != __shm.end(); ++__shmit) {
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
    __logger->log_error(name(), "Cannot convert image data");
    __logger->log_error(name(), e);
  }
  __tt->ping_start(__ttc_dispose);
  __camera->dispose_buffer();
  __tt->ping_end(__ttc_dispose);

  if ( (++__loop_count % FVBASE_TT_PRINT_INT) == 0 ) {
    __tt->print_to_stdout();
  }

#else // no time tracking
  try {
    __camera->capture();
    if ( __enabled ) {
      for (__shmit = __shm.begin(); __shmit != __shm.end(); ++__shmit) {
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
    __logger->log_error(name(), e);
  }
  __camera->dispose_buffer();
#endif

  if ( __mode == AqtCyclic ) {
    vision_threads->wakeup_and_wait_cyclic_threads();
  }

  // reset to the original cancel state, cancelling is now safe
  set_cancel_state(old_cancel_state);
}
