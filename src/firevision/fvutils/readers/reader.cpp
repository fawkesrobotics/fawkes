
/***************************************************************************
 *  reader.cpp - Reader interface
 *
 *  Generated: Thu Mar 29 01:21:19 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/readers/reader.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Reader <fvutils/readers/reader.h>
 * Image reader interface.
 *
 * @fn void Reader::set_buffer(unsigned char *yuv422planar_buffer)
 * Set buffer that the read image should be written to.
 * @param yuv422planar_buffer buffer to write image to. The reader must ensure
 * that it does the proper conversion (if needed) to YUV 422 planar format.
 *
 * @fn colorspace_t Reader::colorspace()
 * Get colorspace from the just read image.
 * @return colorspace
 *
 * @fn unsigned int Reader::pixel_width()
 * Get width of read image in pixels.
 * @return width of image
 *
 * @fn unsigned int Reader::pixel_height()
 * Get height of read image in pixels.
 * @return height of image
 *
 * @fn void Reader::read()
 * Read data from file.
 */

/** Virtual empty destructor. */
Reader::~Reader()
{
}

} // end namespace firevision
