
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

#ifndef __FIREVISION_UTILS_COMPRESSION_JPEG_COMPRESSOR_H_
#define __FIREVISION_UTILS_COMPRESSION_JPEG_COMPRESSOR_H_

#include <fvutils/compression/imagecompressor.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class JpegImageCompressor : public ImageCompressor {
 public:

  /** JPEG color space. */
  enum JpegColorspace {
    JPEG_CS_RGB,	/**< RGB */
    JPEG_CS_YUV		/**< YUV444 packed */
  };

  /** JPEG color space. */
  enum JpegCompressorImplementation {
    JPEG_CI_LIBJPEG,	/**< Force usage of libjpeg for compression */
    JPEG_CI_MMAL	/**< Force usage of MMAL for compression */
  };

  explicit JpegImageCompressor(unsigned int quality = 80, JpegColorspace jcs = JPEG_CS_RGB);
  explicit JpegImageCompressor(JpegCompressorImplementation impl_type,
			       unsigned int quality = 80, JpegColorspace jcs = JPEG_CS_RGB);
  virtual ~JpegImageCompressor();

  virtual void          set_image_dimensions(unsigned int width, unsigned int height)
  { impl_->set_image_dimensions(width, height); }
  virtual void          set_image_buffer(colorspace_t cspace, unsigned char *buffer)
  { impl_->set_image_buffer(cspace, buffer); }
  virtual void          set_destination_buffer(unsigned char *buf, unsigned int buf_size)
  { impl_->set_destination_buffer(buf, buf_size); }
  virtual size_t        compressed_size()
  { return impl_->compressed_size(); }
  virtual void          set_filename(const char *filename)
  { impl_->set_filename(filename); }
  virtual void          set_compression_destination(ImageCompressor::CompressionDestination cd)
  { impl_->set_compression_destination(cd); }
  virtual bool          supports_compression_destination(ImageCompressor::CompressionDestination cd)
  { return impl_->supports_compression_destination(cd); }
  virtual void          compress()
  { impl_->compress(); }
  virtual size_t        recommended_compressed_buffer_size()
  { return impl_->recommended_compressed_buffer_size(); }
  virtual bool          supports_vflip()
  { return impl_->supports_vflip(); }
  virtual void          set_vflip(bool enable)
  { impl_->set_vflip(enable); }

 private:
  ImageCompressor *impl_;
};

} // end namespace firevision

#endif
