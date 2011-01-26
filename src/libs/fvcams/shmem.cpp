/***************************************************************************
 *  shmem.cpp - Implementation to access images in shared memory
 *
 *  Created: Thu Jan 12 19:43:05 2006
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <core/exception.h>
#include <core/exceptions/system.h>
#include <core/exceptions/software.h>
#include <fvcams/shmem.h>
#include <fvutils/writers/fvraw.h>
#include <fvutils/system/camargp.h>

#include <cstring>
#include <cstdlib>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SharedMemoryCamera <fvcams/shmem.h>
 * Shared memory camera.
 * Camera to retrieve images from a shared memory segment.
 *
 * The camera can operate in a so-called deep-copy mode. In this mode a
 * local internal buffer is created of the size of the image. On capture()
 * the image is copied from the shared memory buffer to the local buffer
 * with the shared memory segment locked for reading. This can be used if
 * the image writing and the image reading processess run asynchronously.
 * While locking would suffice the copying will account for only short
 * locking times so that the interference between the two processes is
 * minimal.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param image_id image ID to open
 * @param deep_copy true to operate in deep-copy mode, false otherwise
 */
SharedMemoryCamera::SharedMemoryCamera(const char *image_id, bool deep_copy)
{
  __image_id = strdup(image_id);
  __deep_copy = deep_copy;

  try {
    init();
  } catch (Exception &e) {
    free(__image_id);
    __image_id = NULL;
    throw;
  }
}


/** Constructor.
 * Take configuration data from camera argument parser. The following
 * options are supported.
 * - image_id=ID, where ID is the image ID
 * @param cap camera argument parser
 */
SharedMemoryCamera::SharedMemoryCamera(const CameraArgumentParser *cap)
{
  __image_id  = NULL;
  __deep_copy = false;

  if ( cap->has("image_id") ) {
    __image_id = strdup(cap->get("image_id").c_str());
  }
  else throw MissingParameterException("The parameter 'image_id' is required");

  if ( cap->has("deep_copy") ) {
    __deep_copy = (strcasecmp(cap->get("deep_copy").c_str(), "true") == 0);
  }

  try {
    init();
  } catch (Exception &e) {
    free(__image_id);
    __image_id = NULL;
    throw;
  }
}


/** Destructor. */
SharedMemoryCamera::~SharedMemoryCamera()
{
  free(__image_id);
  if ( __deep_buffer != NULL ) {
    free( __deep_buffer );
  }
  delete __shm_buffer;
  delete __capture_time;
}


void
SharedMemoryCamera::init()
{
  __deep_buffer  = NULL;
  __capture_time = NULL;
  try {
    __shm_buffer = new SharedMemoryImageBuffer(__image_id);
    if ( __deep_copy ) {
      __deep_buffer = (unsigned char *)malloc(__shm_buffer->data_size());
      if ( ! __deep_buffer ) {
	throw OutOfMemoryException("SharedMemoryCamera: Cannot allocate deep buffer");
      }
    }
    __opened = true;
  } catch (Exception &e) {
    e.append("Failed to open shared memory image");
    throw;
  }
  __capture_time = new fawkes::Time(0, 0);
}

void
SharedMemoryCamera::open()
{
}


void
SharedMemoryCamera::start()
{
}

void
SharedMemoryCamera::stop()
{
}

void
SharedMemoryCamera::print_info()
{
}

void
SharedMemoryCamera::capture()
{
  if ( __deep_copy ) {
    __shm_buffer->lock_for_read();
    memcpy(__deep_buffer, __shm_buffer->buffer(), __shm_buffer->data_size());
    __capture_time->set_time(__shm_buffer->capture_time());
    __shm_buffer->unlock();
  }
  else __capture_time->set_time(__shm_buffer->capture_time());
}

unsigned char*
SharedMemoryCamera::buffer()
{
  if ( __deep_copy ) {
    return __deep_buffer;
  } else {
    return __shm_buffer->buffer();
  }
}

unsigned int
SharedMemoryCamera::buffer_size()
{
  return colorspace_buffer_size(__shm_buffer->colorspace(),
				__shm_buffer->width(),
				__shm_buffer->height() );
}

void
SharedMemoryCamera::close()
{
}

void
SharedMemoryCamera::dispose_buffer()
{
}

unsigned int
SharedMemoryCamera::pixel_width()
{
  return __shm_buffer->width();
}

unsigned int
SharedMemoryCamera::pixel_height()
{
  return __shm_buffer->height();
}


colorspace_t
SharedMemoryCamera::colorspace()
{
  return __shm_buffer->colorspace();
}


fawkes::Time *
SharedMemoryCamera::capture_time()
{
  return __capture_time;
}


void
SharedMemoryCamera::flush()
{
}


/** Get the shared memory image buffer.
 * @return shared memory image buffer used to access image
 */
SharedMemoryImageBuffer *
SharedMemoryCamera::shared_memory_image_buffer()
{
  return __shm_buffer;
}


bool
SharedMemoryCamera::ready()
{
  return __opened;
}


void
SharedMemoryCamera::set_image_number(unsigned int n)
{
  // ignore for now
}


/** Lock image for reading.
 * Aquire the lock to read images.
 */
void
SharedMemoryCamera::lock_for_read()
{
  __shm_buffer->lock_for_read();
}


/** Try to lock for reading.
 * @return true if the lock has been aquired, false otherwise
 */
bool
SharedMemoryCamera::try_lock_for_read()
{
  return __shm_buffer->try_lock_for_read();
}


/** Lock image for writing.
 * Aquire the lock to write images.
 */
void
SharedMemoryCamera::lock_for_write()
{
  __shm_buffer->lock_for_write();
}


/** Try to lock for reading.
 * @return true if the lock has been aquired, false otherwise
 */
bool
SharedMemoryCamera::try_lock_for_write()
{
  return __shm_buffer->try_lock_for_write();
}


/** Unlock buffer. */
void
SharedMemoryCamera::unlock()
{
  __shm_buffer->unlock();
}

} // end namespace firevision
