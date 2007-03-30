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
#include <cams/shmem.h>
#include <fvutils/writers/fvraw.h>

/** @class SharedMemoryCamera <cams/shmem.h>
 * Shared memory camera.
 * Camera to retrieve images from a shared memory segment.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param image_num image number to look for.
 */
SharedMemoryCamera::SharedMemoryCamera(unsigned int image_num)
{
  this->image_num = image_num;

  try {
    shm_buffer = new SharedMemoryImageBuffer(image_num);
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
}

unsigned char*
SharedMemoryCamera::buffer()
{
  return shm_buffer->getBuffer();
}

unsigned int
SharedMemoryCamera::buffer_size()
{
  return colorspace_buffer_size(shm_buffer->getColorspace(),
				shm_buffer->getWidth(),
				shm_buffer->getHeight() );
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
  return shm_buffer->getWidth();
}

unsigned int
SharedMemoryCamera::pixel_height()
{
  return shm_buffer->getHeight();
}


colorspace_t
SharedMemoryCamera::colorspace()
{
  return shm_buffer->getColorspace();
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
  if (SharedMemoryImageBuffer::exists(n)) {
    shm_buffer->setImageNumber(n);
  } else {
    throw Exception("Image number does not exist");
  }
}
