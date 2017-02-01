
/***************************************************************************
 *  buffer.cpp - Camera model for a simple buffer
 *
 *  Created: Tue Mar 08 22:44:33 2016
 *  Copyright  2005-2016  Tim Niemueller [www.niemueller.de]
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

#include <fvcams/buffer.h>

#include <cstdlib>


using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BufferCamera <fvcams/buffer.h>
 * Simple buffer with a Camera interface.
 * This camera implementation provides a simple image buffer that can be
 * modified externally and is wrapped using the standard Camera interface.
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * @param cspace color space of image
 * @param width width of image
 * @param height height of image
 */
BufferCamera::BufferCamera(colorspace_t cspace, unsigned int width, unsigned int height)
{
	cspace_ = cspace;
  width_  = width;
  height_ = height;
  buffer_ = malloc_buffer(cspace, width, height);
  buffer_size_ = colorspace_buffer_size(cspace, width, height);
}


/** Destructor. */
BufferCamera::~BufferCamera()
{
	free(buffer_);
}


void
BufferCamera::open()
{
}


void
BufferCamera::start()
{
}

void
BufferCamera::stop()
{
}


void
BufferCamera::print_info()
{
}


void
BufferCamera::capture()
{
}


unsigned char*
BufferCamera::buffer()
{
  return buffer_;
}


unsigned int
BufferCamera::buffer_size()
{
  return buffer_size_;
}


void
BufferCamera::close()
{
}


void
BufferCamera::dispose_buffer()
{
}


void
BufferCamera::flush()
{
}


bool
BufferCamera::ready()
{
	return true;
}


void
BufferCamera::set_image_number(unsigned int n)
{
}


unsigned int
BufferCamera::pixel_width()
{
  return width_;
}


unsigned int
BufferCamera::pixel_height()
{
  return height_;
}


colorspace_t
BufferCamera::colorspace()
{
  return cspace_;
}

} // end namespace firevision
