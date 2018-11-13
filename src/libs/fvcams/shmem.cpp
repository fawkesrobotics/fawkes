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
  image_id_ = strdup(image_id);
  deep_copy_ = deep_copy;

  try {
    init();
  } catch (Exception &e) {
    free(image_id_);
    image_id_ = NULL;
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
  image_id_  = NULL;
  deep_copy_ = false;

  if ( cap->has("image_id") ) {
    image_id_ = strdup(cap->get("image_id").c_str());
  }
  else throw MissingParameterException("The parameter 'image_id' is required");

  if ( cap->has("deep_copy") ) {
    deep_copy_ = (strcasecmp(cap->get("deep_copy").c_str(), "true") == 0);
  }

  try {
    init();
  } catch (Exception &e) {
    free(image_id_);
    image_id_ = NULL;
    throw;
  }
}


/** Destructor. */
SharedMemoryCamera::~SharedMemoryCamera()
{
  free(image_id_);
  if ( deep_buffer_ != NULL ) {
    free( deep_buffer_ );
  }
  delete shm_buffer_;
  delete capture_time_;
}


void
SharedMemoryCamera::init()
{
  deep_buffer_  = NULL;
  capture_time_ = NULL;
  try {
    shm_buffer_ = new SharedMemoryImageBuffer(image_id_);
    if ( deep_copy_ ) {
      deep_buffer_ = (unsigned char *)malloc(shm_buffer_->data_size());
      if ( ! deep_buffer_ ) {
	throw OutOfMemoryException("SharedMemoryCamera: Cannot allocate deep buffer");
      }
    }
    opened_ = true;
  } catch (Exception &e) {
    e.append("Failed to open shared memory image");
    throw;
  }
  capture_time_ = new fawkes::Time(0, 0);
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
  if ( deep_copy_ ) {
    shm_buffer_->lock_for_read();
    memcpy(deep_buffer_, shm_buffer_->buffer(), shm_buffer_->data_size());
    capture_time_->set_time(shm_buffer_->capture_time());
    shm_buffer_->unlock();
  }
  else capture_time_->set_time(shm_buffer_->capture_time());
}

unsigned char*
SharedMemoryCamera::buffer()
{
  if ( deep_copy_ ) {
    return deep_buffer_;
  } else {
    return shm_buffer_->buffer();
  }
}

unsigned int
SharedMemoryCamera::buffer_size()
{
  return colorspace_buffer_size(shm_buffer_->colorspace(),
				shm_buffer_->width(),
				shm_buffer_->height() );
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
  return shm_buffer_->width();
}

unsigned int
SharedMemoryCamera::pixel_height()
{
  return shm_buffer_->height();
}


colorspace_t
SharedMemoryCamera::colorspace()
{
  return shm_buffer_->colorspace();
}


fawkes::Time *
SharedMemoryCamera::capture_time()
{
  return capture_time_;
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
  return shm_buffer_;
}


bool
SharedMemoryCamera::ready()
{
  return opened_;
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
  shm_buffer_->lock_for_read();
}


/** Try to lock for reading.
 * @return true if the lock has been aquired, false otherwise
 */
bool
SharedMemoryCamera::try_lock_for_read()
{
  return shm_buffer_->try_lock_for_read();
}


/** Lock image for writing.
 * Aquire the lock to write images.
 */
void
SharedMemoryCamera::lock_for_write()
{
  shm_buffer_->lock_for_write();
}


/** Try to lock for reading.
 * @return true if the lock has been aquired, false otherwise
 */
bool
SharedMemoryCamera::try_lock_for_write()
{
  return shm_buffer_->try_lock_for_write();
}


/** Unlock buffer. */
void
SharedMemoryCamera::unlock()
{
  shm_buffer_->unlock();
}

} // end namespace firevision
