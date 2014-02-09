
/***************************************************************************
 *  jpeg_compressor_mmal.h - JPEG image compressor interface (using MMAL)
 *
 *  Created: Wed Feb 05 15:12:28 2014
 *  Copyright  2005-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_UTILS_COMPRESSION_JPEG_COMPRESSOR_MMAL_H_
#define __FIREVISION_UTILS_COMPRESSION_JPEG_COMPRESSOR_MMAL_H_

#ifndef __FIREVISION_UTILS_COMPRESSION_JPEG_COMPRESSOR_H_
#  error Do not include jpeg_compressor_mmal.h directly, use jpeg_compressor.h
#endif

#ifndef HAVE_MMAL
#  error Cannot use MMAL JPEG Encoder without MMAL
#endif

#include <fvutils/compression/imagecompressor.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class JpegImageCompressorMMAL : public ImageCompressor {
 public:

  JpegImageCompressorMMAL(unsigned int quality = 80);
  virtual ~JpegImageCompressorMMAL();

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

  class State;

 private:
  void create_encoder_component();
  void destroy_encoder_component();

 private:
  unsigned char *buffer_;

  unsigned int   quality_;

  unsigned int   width_;
  unsigned int   height_;

  const char    *filename_;

  State *state_;
  bool   vflip_;
};

} // end namespace firevision

#endif
