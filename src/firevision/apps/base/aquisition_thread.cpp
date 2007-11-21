
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
#include <apps/base/aqt_vision_threads.h>

#include <core/exceptions/system.h>
#include <utils/time/clock.h>
#include <utils/logging/logger.h>

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
 * @param logger logger
 * @param id id to be used for the shared memory segment and to announce changes
 * to the base thread
 * @param camera camera to manage
 * @param clock clock to use for timeout measurement (system time)
 */
FvAquisitionThread::FvAquisitionThread(const char *id,  Camera *camera,
				       Logger *logger, Clock *clock)
  : Thread((std::string("FvAquisitionThread::") + id).c_str())
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
}


/** Destructor. */
FvAquisitionThread::~FvAquisitionThread()
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
 * @param deep_copy given to the shared memory camera.
 * @return camera instance
 * @see SharedMemoryCamera
 */
SharedMemoryCamera *
FvAquisitionThread::camera_instance(bool raw, bool deep_copy)
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


/** Set aquisition thread mode.
 * Note that this may only be called on a stopped thread or an
 * exception will be thrown by Thread::set_opmode()!
 * @param mode new aquisition thread mode
 */
void
FvAquisitionThread::set_aqtmode(AqtMode mode)
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


/** Get aquisition thread mode.
 * @return aquisition thread mode.
 */
FvAquisitionThread::AqtMode
FvAquisitionThread::aqtmode()
{
  return _mode;
}


void
FvAquisitionThread::loop()
{
  try {
    //_logger->log_debug(name(), "Capture");
    _camera->capture();
    if ( _shm ) {
      //_logger->log_debug(name(), "Convert");
      _shm->lock_for_write();
      convert(_colorspace, YUV422_PLANAR,
	      _camera->buffer(), _buffer,
	      _width, _height);
      _shm->unlock();
    }
    if ( _shm_raw ) {
      //_logger->log_debug(name(), "Copy");
      _shm_raw->lock_for_write();
      memcpy(_buffer_raw, _camera->buffer(), _camera->buffer_size());
      _shm_raw->unlock();
    }
  } catch (Exception &e) {
    //_logger->log_error(name(), "Cannot convert image data");
    _logger->log_error(name(), e);
  }
  _camera->dispose_buffer();

  //_logger->log_debug(name(), "Wake");
  _vision_threads->wakeup_cyclic_threads();
  //_logger->log_debug(name(), "Wait");
  _vision_threads->wait_cyclic_threads();
  //_logger->log_debug(name(), "Done");
}
