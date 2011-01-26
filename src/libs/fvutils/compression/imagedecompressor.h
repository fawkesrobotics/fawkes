
/***************************************************************************
 *  imagedecompressor.h - image de-compressor interface
 *
 *  Created: Tue Nov 13 10:50:12 2007
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

#ifndef __FIREVISION_UTILS_COMPRESSION_IMAGEDECOMPRESSOR_H_
#define __FIREVISION_UTILS_COMPRESSION_IMAGEDECOMPRESSOR_H_

#include <fvutils/color/colorspaces.h>
#include <sys/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ImageDecompressor {
 public:

  /* Maybe later...
  * Where to get the compressed image from
  enum DecompressionSource {
    COMP_SRC_FILE,	< read compressed image from file
    COMP_SRC_MEM	< read compressed image from buffer in memory
  };
  */

  virtual ~ImageDecompressor();

  virtual void          set_image_dimensions(unsigned int width, unsigned int height);
  virtual void          set_compressed_buffer(unsigned char *buf, unsigned int buf_size);
  virtual void          set_decompressed_buffer(unsigned char *buf, unsigned int buf_size);
  virtual void          decompress() = 0;

 protected:
  unsigned int    _width;
  unsigned int    _height;
  unsigned char * _compressed_buffer;
  unsigned char * _decompressed_buffer;
  unsigned int    _compressed_buffer_size;
  unsigned int    _decompressed_buffer_size;

};

} // end namespace firevision

#endif
