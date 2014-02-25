
/***************************************************************************
 *  jpeg_compressor.h - JPEG image compressor interface
 *
 *  Created: Fri Aug 11 18:53:19 2006 (on train to Cologne)
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

#ifndef __FIREVISION_UTILS_COMPRESSION_JPEG_COMPRESSOR_LIBJPEG_H_
#define __FIREVISION_UTILS_COMPRESSION_JPEG_COMPRESSOR_LIBJPEG_H_

#ifndef __FIREVISION_UTILS_COMPRESSION_JPEG_COMPRESSOR_H_
#  error Do not include jpeg_compressor_libjpeg.h directly, use jpeg_compressor.h
#endif

#include <fvutils/compression/imagecompressor.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class JpegImageCompressorLibJpeg : public ImageCompressor {
 public:

  JpegImageCompressorLibJpeg(unsigned int quality = 80,
			     JpegImageCompressor::JpegColorspace jcs = JpegImageCompressor::JPEG_CS_RGB);
  virtual ~JpegImageCompressorLibJpeg();

  virtual void          set_image_dimensions(unsigned int width, unsigned int height);
  virtual void          set_image_buffer(colorspace_t cspace, unsigned char *buffer);
  virtual void          set_destination_buffer(unsigned char *buf, unsigned int buf_size);
  virtual size_t        compressed_size();
  virtual void          set_filename(const char *filename);
  virtual void          set_compression_destination(ImageCompressor::CompressionDestination cd);
  virtual bool          supports_compression_destination(ImageCompressor::CompressionDestination cd);
  virtual void          compress();
  virtual size_t        recommended_compressed_buffer_size();
  virtual bool          supports_vflip();
  virtual void          set_vflip(bool enable);

 private:
  unsigned char *jpeg_buffer;
  unsigned int   jpeg_buffer_size;
  unsigned char *buffer;

  unsigned int   quality;
  bool           vflip;

  unsigned int   width;
  unsigned int   height;

  unsigned int   jpeg_bytes;

  const char    *filename;

  JpegImageCompressor::JpegColorspace jpeg_cs;
  CompressionDestination compdest;
};

} // end namespace firevision

#endif
