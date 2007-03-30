
/***************************************************************************
 *  imagecompressor.h - image compressor interface
 *
 *  Generated: Fri Aug 11 18:53:19 2006 (on train to Cologne)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_UTILS_COMPRESSION_JPEG_H_
#define __FIREVISION_UTILS_COMPRESSION_JPEG_H_

#include <fvutils/compression/imagecompressor.h>

class JpegImageCompressor : public ImageCompressor {
 public:

  /** JPEG color space. */
  enum JpegColorspace {
    JPEG_CS_RGB,	/**< RGB */
    JPEG_CS_YUV		/**< YUV444 packed */
  };

  JpegImageCompressor(JpegColorspace jcs = JPEG_CS_RGB);
  virtual ~JpegImageCompressor();

  virtual void          set_image_dimensions(unsigned int width, unsigned int height);
  virtual void          set_image_buffer(colorspace_t cspace, unsigned char *buffer);
  virtual void          set_destination_buffer(unsigned char *buf, unsigned int buf_size);
  virtual unsigned int  compressed_size();
  virtual void          set_filename(const char *filename);
  virtual void          set_compression_destination(ImageCompressor::CompressionDestination cd);
  virtual bool          supports_compression_destination(ImageCompressor::CompressionDestination cd);
  virtual void          compress();

 private:
  unsigned char *jpeg_buffer;
  unsigned int   jpeg_buffer_size;
  unsigned char *buffer;

  unsigned int   quality;

  unsigned int   width;
  unsigned int   height;

  unsigned int   jpeg_bytes;

  const char    *filename;

  JpegColorspace jpeg_cs;
  CompressionDestination compdest;
};


#endif
