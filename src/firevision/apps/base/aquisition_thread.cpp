
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
  _base_thread = base_thread;
  _logger      = logger;
  _image_id    = strdup(id);
  _camera      = camera;
  _timeout     = timeout;
  _width       = _camera->pixel_width();
  _height      = _camera->pixel_height();
  _colorspace  = _camera->colorspace();
  _shm = new SharedMemoryImageBuffer(_image_id, YUV422_PLANAR,
				     _width, _height);
  _buffer      = _shm->buffer();

  _running_tl         = new ThreadList();
  _running_tl_barrier = NULL;
  _waiting_tl         = new ThreadList();

  _wait_for_threads_mutex = new Mutex();
  _wait_for_threads_cond  = new WaitCondition();
}


/** Destructor. */
FvAquisitionThread::~FvAquisitionThread()
{
  _camera->close();

  delete _shm;
  delete _camera;
  delete _running_tl;
  delete _running_tl_barrier;
  delete _waiting_tl;
  delete _wait_for_threads_cond;
  delete _wait_for_threads_mutex;
  free(_image_id);
}


/** Get a camera instace.
 * This will return a camera instance suitable for accessing the image
 * buffer. Note, that this is not the camera provided to the constructor,
 * but rather a SharedMemoryCamera instance accessing a shared memory buffer
 * where the image is copied to (or a conversion result is posted to).
 * The returned instance has to bee freed using delete when done with it.
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
 * @return camera instance
 */
Camera *
FvAquisitionThread::camera_instance()
{
  return new SharedMemoryCamera(_image_id);
}


/** Check if aquisition thread maintains given thread.
 * @param thread thread to check for
 * @return true, if the thread is managed by this aquisition thread, false otherwise
 */
bool
FvAquisitionThread::has_thread(Thread *thread)
{
  return ( (find(_running_tl->begin(), _running_tl->end(), thread) != _running_tl->end()) ||
	   (find(_waiting_tl->begin(), _waiting_tl->end(), thread) != _waiting_tl->end()) );
}


/** Add a thread.
 * Adds a thread to the waiting queue and registeres to the thread's notification.
 * @param thread thread to add
 */
void
FvAquisitionThread::add_thread(Thread *thread)
{
  _waiting_tl->lock();
  VisionAspect *vision_thread;
  if ( (vision_thread = dynamic_cast<VisionAspect *>(thread)) != NULL ) {
    _waiting_tl->push_back(thread);
    thread->add_notification_listener(this);
  } else {
    throw Exception("Thread does not have the VisionAspect");
  }
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
  _waiting_tl->lock();
  VisionAspect *vision_thread;
  if ( (vision_thread = dynamic_cast<VisionAspect *>(thread)) != NULL ) {
    _running_tl->remove(thread);
    delete _running_tl_barrier;
    _running_tl_barrier = new Barrier(_running_tl->size() + 1);
    _waiting_tl->remove(thread);
  } else {
    throw Exception("Thread does not have the VisionAspect");
  }
  _waiting_tl->unlock();
  _running_tl->unlock();
}


void
FvAquisitionThread::thread_started(Thread *thread)
{
  _running_tl->push_back_locked(thread);
  _waiting_tl->remove_locked(thread);
  _wait_for_threads_cond->wakeAll();
}


void
FvAquisitionThread::thread_init_failed(Thread *thread)
{
  _waiting_tl->remove_locked(thread);
  _wait_for_threads_cond->wakeAll();
}


/** Check if queues are empty.
 * @return true if running and waiting queues are empty, false otherwise
 */
bool
FvAquisitionThread::empty()
{
  return (_running_tl->empty() && _waiting_tl->empty());
}


void
FvAquisitionThread::loop()
{
  _running_tl->lock();
  if ( _running_tl->empty() ) {
    _running_tl->unlock();
    _wait_for_threads_mutex->lock();
    _wait_for_threads_cond->wait(_wait_for_threads_mutex, _timeout);
    _wait_for_threads_mutex->unlock();
    _running_tl->lock();
  }
  if (empty()) {
    _base_thread->aqt_timeout(_image_id);
    _running_tl->unlock();
    return;
  } else if ( _running_tl->empty() ) {
    _running_tl->unlock();
    return;
  }

  _camera->capture();
  // Always do this?
  try {
    convert(_colorspace, YUV422_PLANAR,
	    _camera->buffer(), _buffer,
	    _width, _height);

    _running_tl->wakeup(_running_tl_barrier);
    _running_tl_barrier->wait();
    _running_tl->unlock();

  } catch (Exception &e) {
    _logger->log_error(name(), "Cannot convert image data");
    _logger->log_error(name(), e);
  }

}
