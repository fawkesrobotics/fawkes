/***************************************************************************
 *  shmem.cpp - Implementation to access images in shared memory
 *
 *  Generated: Thu Jan 12 19:43:05 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>
#include <core/exceptions/system.h>
#include <cams/shmem.h>
#include <fvutils/writers/fvraw.h>
#include <fvutils/system/camargp.h>

/** @class SharedMemoryCamera <cams/shmem.h>
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
  this->image_id = strdup(image_id);
  this->deep_copy = deep_copy;

  try {
    init();
  } catch (Exception &e) {
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
  if ( cap->has("image_id") ) {
    image_id = strdup(cap->get("image_id").c_str());
  }
  try {
    init();
  } catch (Exception &e) {
    throw;
  }
}


/** Destructor. */
SharedMemoryCamera::~SharedMemoryCamera()
{
  free(image_id);
  if ( deep_buffer != NULL ) {
    free( deep_buffer );
  }
}


void
SharedMemoryCamera::init()
{
  deep_buffer = NULL;
  try {
    shm_buffer = new SharedMemoryImageBuffer(image_id);
    if ( deep_copy ) {
      deep_buffer = (unsigned char *)malloc(shm_buffer->data_size());
      if ( ! deep_buffer ) {
	throw OutOfMemoryException("SharedMemoryCamera: Cannot allocate deep buffer");
      }
    }
    opened = true;
  } catch (Exception &e) {
    e.append("Failed to open shared memory image");
    throw;
  }
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
  if ( deep_copy ) {
    shm_buffer->lock_for_read();
    memcpy(deep_buffer, shm_buffer->buffer(), shm_buffer->data_size());
    shm_buffer->unlock();
  }
}

unsigned char*
SharedMemoryCamera::buffer()
{
  if ( deep_copy ) {
    return deep_buffer;
  } else {
    return shm_buffer->buffer();
  }
}

unsigned int
SharedMemoryCamera::buffer_size()
{
  return colorspace_buffer_size(shm_buffer->colorspace(),
				shm_buffer->width(),
				shm_buffer->height() );
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
  return shm_buffer->width();
}

unsigned int
SharedMemoryCamera::pixel_height()
{
  return shm_buffer->height();
}


colorspace_t
SharedMemoryCamera::colorspace()
{
  return shm_buffer->colorspace();
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
  return shm_buffer;
}


bool
SharedMemoryCamera::ready()
{
  return opened;
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
  shm_buffer->lock_for_read();
}


/** Try to lock for reading.
 * @return true if the lock has been aquired, false otherwise
 */
bool
SharedMemoryCamera::try_lock_for_read()
{
  return shm_buffer->try_lock_for_read();
}


/** Lock image for writing.
 * Aquire the lock to write images.
 */
void
SharedMemoryCamera::lock_for_write()
{
  shm_buffer->lock_for_write();
}


/** Try to lock for reading.
 * @return true if the lock has been aquired, false otherwise
 */
bool
SharedMemoryCamera::try_lock_for_write()
{
  return shm_buffer->try_lock_for_write();
}


/** Unlock buffer. */
void
SharedMemoryCamera::unlock()
{
  shm_buffer->unlock();
}
