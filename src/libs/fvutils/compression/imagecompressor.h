
/***************************************************************************
 *  imagecompressor.h - image compressor interface
 *
 *  Generated: Fri Aug 11 18:53:19 2006 (on train to Cologne)
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

#ifndef __FIREVISION_UTILS_COMPRESSION_IMAGE_COMPRESSOR_H_
#define __FIREVISION_UTILS_COMPRESSION_IMAGE_COMPRESSOR_H_

#include <fvutils/color/colorspaces.h>
#include <sys/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ImageCompressor {
 public:

  /** Where to put the compressed image */
  enum CompressionDestination {
    COMP_DEST_FILE,	/**< write compressed image to file */
    COMP_DEST_MEM	/**< write compressed image to buffer in memory */
  };

  virtual ~ImageCompressor();

  virtual void          set_image_dimensions(unsigned int width, unsigned int height)     = 0;
  virtual void          set_image_buffer(colorspace_t cspace, unsigned char *buffer)      = 0;
  virtual void          set_destination_buffer(unsigned char *buf, unsigned int buf_size) = 0;
  virtual size_t        compressed_size()                                                 = 0;
  virtual void          set_filename(const char *filename)                                = 0;
  virtual void          set_compression_destination(CompressionDestination cd)            = 0;
  virtual bool          supports_compression_destination(CompressionDestination cd)       = 0;
  virtual size_t        recommended_compressed_buffer_size()                              = 0;
  virtual void          compress()                                                        = 0;

  virtual bool          supports_vflip() = 0;
  virtual void          set_vflip(bool enable) = 0;
};

} // end namespace firevision

#endif
