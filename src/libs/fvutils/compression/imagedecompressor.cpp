
/***************************************************************************
 *  imagedecompressor.cpp - image de-compressor interface
 *
 *  Created: Tue Nov 13 10:54:03 2007
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

#include <fvutils/compression/imagedecompressor.h>


namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ImageDecompressor <fvutils/compression/imagedecompressor.h>
 * Image de-compressor interface.
 * Currently only decompressing from memory to memory is supported.
 * @author Tim Niemueller
 *
 * @fn void ImageDecompressor::decompress()
 * Decompress image.
 */

/** @var int ImageDecompressor::_width
 * Width of image in pixels
 */

/** @var int ImageDecompressor::_height
 * Height of image in pixels
 */

/** @var int ImageDecompressor::_compressed_buffer
 * Buffer containing the compressed image
 */

/** @var int ImageDecompressor::_compressed_buffer_size
 * Size in bytes of _compressed_buffer
 */

/** @var int ImageDecompressor::_decompressed_buffer
 * Buffer containing the decompressed image after decompression
 */

/** @var int ImageDecompressor::_decompressed_buffer_size
 * Size in bytes of _decompressed_buffer
 */


/** Virtual empty destructor. */
ImageDecompressor::~ImageDecompressor()
{
}


/** Set image dimensions.
 * @param width width of image in pixels
 * @param height height of image in pixels
 */
void
ImageDecompressor::set_image_dimensions(unsigned int width, unsigned int height)
{
  _width  = width;
  _height = height;
}


/** Set compressed buffer.
 * @param buf buffer
 * @param buf_size size of buffer in bytes
 */
void
ImageDecompressor::set_compressed_buffer(unsigned char *buf, unsigned int buf_size)
{
  _compressed_buffer = buf;
  _compressed_buffer_size = buf_size;
}


/** Set decompressed buffer.
 * @param buf decompressed buffer
 * @param buf_size buffer size
 */
void
ImageDecompressor::set_decompressed_buffer(unsigned char *buf, unsigned int buf_size)
{
  _decompressed_buffer = buf;
  _decompressed_buffer_size = buf_size;
}

} // end namespace firevision
