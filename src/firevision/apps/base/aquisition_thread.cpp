
/***************************************************************************
 *  aquisition_thread.h - FireVision Aquisition Thread
 *
 *  Created: Wed Jun 06 19:01:10 2007
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

#include <apps/base/aquisition_thread.h>
#include <apps/base/base_thread.h>

#include <core/exceptions/system.h>
#include <core/threading/thread_list.h>
#include <core/threading/wait_condition.h>
#include <core/threading/mutex.h>
#include <core/threading/barrier.h>
#include <aspect/vision.h>
#include <cams/shmem.h>
#include <fvutils/color/conversions.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <cstdio>
#include <string>
#include <algorithm>

/** @class FvAquisitionThread <apps/base/aquisition_thread.h>
 * FireVision base application aquisition thread.
 * This thread is used by the base application to aquire images from a camera
 * and call dependant threads when new images are available so that these
 * threads can start processing the images.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param base_thread parent base thread
 * @param logger logger
 * @param id id to be used for the shared memory segment and to announce changes
 * to the base thread
 * @param camera camera to manage
 * @param timeout aquisition timeout, if there is no dependant thread for
 * this amount of time the base thread is signaled to stop this aquisition thread.
 */
FvAquisitionThread::FvAquisitionThread(FvBaseThread *base_thread, Logger *logger,
				       const char *id,  Camera *camera,
				       unsigned int timeout)
  : Thread((std::string("FvAquisitionThread::") + id).c_str())
{
  _base_thread   = base_thread;
  _logger        = logger;
  _image_id      = strdup(id);
  if ( asprintf(&_image_id_raw, "%s::RAW", _image_id) == -1 ) {
    throw OutOfMemoryException();
  }
  _camera        = camera;
  _timeout       = timeout;
  _width         = _camera->pixel_width();
  _height        = _camera->pixel_height();
  _colorspace    = _camera->colorspace();
  logger->log_debug(name(), "Camera opened, w=%u  h=%u  c=%s", _width, _height,
		    colorspace_to_string(_colorspace));
  _shm         = NULL;
  _shm_raw     = NULL;
  _buffer      = NULL;
  _buffer_raw  = NULL;

  _running_tl             = new ThreadList();
  _running_tl_barrier     = NULL;
  _waiting_tl             = new ThreadList();

  _running_raw_tl         = new ThreadList();
  _running_raw_tl_barrier = NULL;
  _waiting_raw_tl         = new ThreadList();

  _wait_for_threads_mutex = new Mutex();
  _wait_for_threads_cond  = new WaitCondition();
}


/** Destructor. */
FvAquisitionThread::~FvAquisitionThread()
{
  _camera->close();

  delete _shm;
  delete _shm_raw;
  delete _camera;
  delete _running_tl;
  delete _running_tl_barrier;
  delete _waiting_tl;
  delete _running_raw_tl;
  delete _running_raw_tl_barrier;
  delete _waiting_raw_tl;
  delete _wait_for_threads_cond;
  delete _wait_for_threads_mutex;
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
 * initialization may even fail. For this reason the aquisition thread
 * registers itself to receive status notifications of the thread. If the
 * thread signals successful startup it is moved to the running queue and
 * from then on woken up when new image material can be processed. If the
 * thread fails for whatever reason it is dropped.
 *
 * The aquisition thread has a timeout. If no thread is in the running or
 * waiting queue for this number of seconds, the base thread is signalled
 * to shut down this aquisition thread (which the base thread may do or
 * deny). This is done so that if a plugin is just unloaded shortly and
 * then quickly loaded again the overhead of closing the camera and then
 * opening it again is avoided.
 *
 * @param raw true to get access to the raw camera image and not the (maybe)
 * converted YUV422_PLANAR image.
 * @return camera instance
 */
Camera *
FvAquisitionThread::camera_instance(bool raw)
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
    return new SharedMemoryCamera(_image_id_raw);
  } else {
    return new SharedMemoryCamera(_image_id);
  }
}


/** Check if aquisition thread maintains given thread.
 * @param thread thread to check for
 * @return true, if the thread is managed by this aquisition thread, false otherwise
 */
bool
FvAquisitionThread::has_thread(Thread *thread)
{
  return ( (find(_running_tl->begin(), _running_tl->end(), thread) != _running_tl->end()) ||
	   (find(_waiting_tl->begin(), _waiting_tl->end(), thread) != _waiting_tl->end()) ||
	   (find(_running_raw_tl->begin(), _running_raw_tl->end(), thread) != _running_raw_tl->end()) ||
	   (find(_waiting_raw_tl->begin(), _waiting_raw_tl->end(), thread) != _waiting_raw_tl->end()) );
}


/** Add a thread.
 * Adds a thread to the waiting queue and registeres to the thread's notification.
 * @param thread thread to add
 * @param raw true if this thread accesses the camera in raw mode, false otherwise
 */
void
FvAquisitionThread::add_thread(Thread *thread, bool raw)
{
  _waiting_tl->lock();
  _waiting_raw_tl->lock();
  VisionAspect *vision_thread;
  if ( (vision_thread = dynamic_cast<VisionAspect *>(thread)) != NULL ) {
    if ( raw ) {
      _waiting_raw_tl->push_back(thread);
    } else {
      _waiting_tl->push_back(thread);
    }
    thread->add_notification_listener(this);
  } else {
    throw Exception("Thread does not have the VisionAspect");
  }
  _waiting_raw_tl->unlock();
  _waiting_tl->unlock();
}



/** Remove a thread.
 * Removes a thread from the running and waiting queues.
 * @param thread thread to remove
 */
void
FvAquisitionThread::remove_thread(Thread *thread)
{
  _running_tl->lock();
  _running_raw_tl->lock();
  _waiting_tl->lock();
  _waiting_raw_tl->lock();
  _logger->log_debug(name(), "Removing thread %s", thread->name());
  VisionAspect *vision_thread;
  if ( (vision_thread = dynamic_cast<VisionAspect *>(thread)) != NULL ) {
    _running_tl->remove(thread);
    _running_raw_tl->remove(thread);
    delete _running_tl_barrier;
    delete _running_raw_tl_barrier;
    _running_tl_barrier = new Barrier(_running_tl->size() + 1);
    _running_raw_tl_barrier = new Barrier(_running_raw_tl->size() + 1);
    _waiting_tl->remove(thread);
    _waiting_raw_tl->remove(thread);
  } else {
    throw Exception("Thread does not have the VisionAspect");
  }
  _waiting_raw_tl->unlock();
  _waiting_tl->unlock();
  _running_raw_tl->unlock();
  _running_tl->unlock();
}


void
FvAquisitionThread::thread_started(Thread *thread)
{
  _running_tl->lock();
  _running_raw_tl->lock();
  if ( find(_waiting_tl->begin(), _waiting_tl->end(), thread) != _waiting_tl->end() ) {
    _running_tl->push_back(thread);
    _waiting_tl->remove_locked(thread);
    delete _running_tl_barrier;
    _running_tl_barrier = new Barrier(_running_tl->size() + 1);
  } else if ( find(_waiting_raw_tl->begin(), _waiting_raw_tl->end(), thread) != _waiting_raw_tl->end() ) {
    _running_raw_tl->push_back(thread);
    _waiting_raw_tl->remove_locked(thread);
    delete _running_raw_tl_barrier;
    _running_raw_tl_barrier = new Barrier(_running_raw_tl->size() + 1);
  }
  _running_raw_tl->unlock();
  _running_tl->unlock();
  _wait_for_threads_cond->wakeAll();
}


void
FvAquisitionThread::thread_init_failed(Thread *thread)
{
  _waiting_tl->remove_locked(thread);
  _waiting_raw_tl->remove_locked(thread);
  _wait_for_threads_cond->wakeAll();
}


/** Check if queues are empty.
 * @return true if running and waiting queues are empty, false otherwise
 */
bool
FvAquisitionThread::empty()
{
  return (_running_tl->empty() && _waiting_tl->empty() &&
	  _running_raw_tl->empty() && _waiting_raw_tl->empty() );
}


void
FvAquisitionThread::loop()
{
  _running_tl->lock();
  _running_raw_tl->lock();
  if ( _running_tl->empty() && _running_raw_tl->empty() ) {
    _running_tl->unlock();
    _running_raw_tl->unlock();
    _wait_for_threads_mutex->lock();
    _logger->log_debug(name(), "Waiting for threads or timeout");
    _wait_for_threads_cond->wait(_wait_for_threads_mutex, _timeout);
    _wait_for_threads_mutex->unlock();
    _running_tl->lock();
    _running_raw_tl->lock();
  }
  if (empty()) {
    _logger->log_debug(name(), "Signaling timeout %s", _image_id);
    _base_thread->aqt_timeout(_image_id);
    _running_tl->unlock();
    _running_raw_tl->unlock();
    exit();
  } else if ( _running_tl->empty() && _running_raw_tl->empty() ) {
    _running_tl->unlock();
    _running_raw_tl->unlock();
    return;
  }

  _camera->capture();
  // Always do this?
  try {
    if ( ! _running_tl->empty() ) {
      convert(_colorspace, YUV422_PLANAR,
	      _camera->buffer(), _buffer,
	      _width, _height);
      //_logger->log_debug(name(), "Waking threads (convert)");
      _running_tl->wakeup_unlocked(_running_tl_barrier);
    }
    if ( ! _running_raw_tl->empty() ) {
      memcpy(_buffer_raw, _camera->buffer(), _camera->buffer_size());
      //_logger->log_debug(name(), "Waking threads (memcpy)");
      _running_raw_tl->wakeup_unlocked(_running_raw_tl_barrier);
    }

    //_logger->log_debug(name(), "Waiting for threads");
    if ( ! _running_tl->empty() )      _running_tl_barrier->wait();
    if ( ! _running_raw_tl->empty() )  _running_raw_tl_barrier->wait();
    _running_raw_tl->unlock();
    _running_tl->unlock();

  } catch (Exception &e) {
    _camera->dispose_buffer();
    _logger->log_error(name(), "Cannot convert image data");
    _logger->log_error(name(), e);
  }
  _camera->dispose_buffer();
}
