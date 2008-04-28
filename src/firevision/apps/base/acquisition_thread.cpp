
/***************************************************************************
 *  acquisition_thread.h - FireVision Acquisition Thread
 *
 *  Created: Wed Jun 06 19:01:10 2007
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

#include <apps/base/acquisition_thread.h>
#include <apps/base/aqt_vision_threads.h>

#include <core/exceptions/system.h>
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

/** @class FvAcquisitionThread <apps/base/acquisition_thread.h>
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
  _logger        = logger;
  _image_id      = strdup(id);
  if ( asprintf(&_image_id_raw, "%s::RAW", _image_id) == -1 ) {
    throw OutOfMemoryException();
  }

  _vision_threads = new FvAqtVisionThreads(clock);

  _camera        = camera;
  _width         = _camera->pixel_width();
  _height        = _camera->pixel_height();
  _colorspace    = _camera->colorspace();
  logger->log_debug(name(), "Camera opened, w=%u  h=%u  c=%s", _width, _height,
		    colorspace_to_string(_colorspace));
  _shm         = NULL;
  _shm_raw     = NULL;
  _buffer      = NULL;
  _buffer_raw  = NULL;

  _mode = AqtContinuous;

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
  _camera->close();

  delete _vision_threads;

  delete _shm;
  delete _shm_raw;
  delete _camera;
  free(_image_id);
  free(_image_id_raw);
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
 * @param raw true to get access to the raw camera image and not the (maybe)
 * converted YUV422_PLANAR image.
 * @param deep_copy given to the shared memory camera.
 * @return camera instance
 * @see SharedMemoryCamera
 */
SharedMemoryCamera *
FvAcquisitionThread::camera_instance(bool raw, bool deep_copy)
{
  if ( raw && (_shm_raw == NULL) ) {
    _shm_raw = new SharedMemoryImageBuffer(_image_id_raw, _colorspace,
					   _width, _height);
    _buffer_raw = _shm_raw->buffer();
  } else if ( _shm == NULL ) {
    _shm = new SharedMemoryImageBuffer(_image_id, YUV422_PLANAR,
				       _width, _height);
    _buffer = _shm->buffer();
  }

  if ( raw ) {
    return new SharedMemoryCamera(_image_id_raw, deep_copy);
  } else {
    return new SharedMemoryCamera(_image_id, deep_copy);
  }
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
    //_logger->log_info(name(), "Setting WAITFORWAKEUPMODE");
    set_opmode(Thread::OPMODE_WAITFORWAKEUP);
  } else if ( mode == AqtContinuous ) {
    //_logger->log_info(name(), "Setting CONTINUOUS");
    set_opmode(Thread::OPMODE_CONTINUOUS);
  }
  _mode = mode;
}


/** Get acquisition thread mode.
 * @return acquisition thread mode.
 */
FvAcquisitionThread::AqtMode
FvAcquisitionThread::aqtmode()
{
  return _mode;
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
    _camera->capture();
    __tt->ping_end(__ttc_capture);
    if ( _shm ) {
      __tt->ping_start(__ttc_lock);
      _shm->lock_for_write();
      __tt->ping_end(__ttc_lock);
      __tt->ping_start(__ttc_convert);
      convert(_colorspace, YUV422_PLANAR,
	      _camera->buffer(), _buffer,
	      _width, _height);
      __tt->ping_end(__ttc_convert);
      __tt->ping_start(__ttc_unlock);
      _shm->unlock();
      __tt->ping_end(__ttc_unlock);
    }
    if ( _shm_raw ) {
      _shm_raw->lock_for_write();
      memcpy(_buffer_raw, _camera->buffer(), _camera->buffer_size());
      _shm_raw->unlock();
    }
  } catch (Exception &e) {
    _logger->log_error(name(), "Cannot convert image data");
    _logger->log_error(name(), e);
  }
  __tt->ping_start(__ttc_dispose);
  _camera->dispose_buffer();
  __tt->ping_end(__ttc_dispose);

  if ( (++__loop_count % FVBASE_TT_PRINT_INT) == 0 ) {
    __tt->print_to_stdout();
  }

#else // no time tracking
  try {
    _camera->capture();
    if ( _shm ) {
      _shm->lock_for_write();
      convert(_colorspace, YUV422_PLANAR,
	      _camera->buffer(), _buffer,
	      _width, _height);
      _shm->unlock();
    }
    if ( _shm_raw ) {
      _shm_raw->lock_for_write();
      memcpy(_buffer_raw, _camera->buffer(), _camera->buffer_size());
      _shm_raw->unlock();
    }
  } catch (Exception &e) {
    _logger->log_error(name(), e);
  }
  _camera->dispose_buffer();
#endif

  _vision_threads->wakeup_cyclic_threads();
  _vision_threads->wait_cyclic_threads();

  // reset to the original cancel state, cancelling is now safe
  set_cancel_state(old_cancel_state);
}
