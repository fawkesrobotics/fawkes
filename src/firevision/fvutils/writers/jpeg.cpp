
/***************************************************************************
 *  jpeg.cp - JPEG writer
 *
 *  Generated: Wed Jun 28 11:36:54 2006 (my brother's 18th birthday)
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>
#include <fvutils/writers/jpeg.h>
#include <fvutils/color/yuvrgb.h>

#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string.h>
extern "C" {
#include <jpeglib.h>
}

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class JpegWriter <fvutils/writers/jpeg.h>
 * JPEG file writer.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param quality quality, value between 0 and 100
 */
JpegWriter::JpegWriter(int quality)
  : Writer("jpg")
{
  buffer = NULL;
  
  this->quality  = (quality > 0) ? quality : -quality;
}

/** Constructor.
 * @param filename file name to write to
 * @param quality quality, value between 0 and 100
 */
JpegWriter::JpegWriter(const char *filename, int quality)
  : Writer("jpg")
{
  set_filename(filename);

  buffer = NULL;

  this->quality  = (quality > 0) ? quality : -quality;
}


/** Destructor. */
JpegWriter::~JpegWriter()
{
}


void
JpegWriter::set_buffer(colorspace_t cspace, unsigned char *buffer)
{
  if (cspace == YUV422_PLANAR) {
    this->buffer = buffer;
  } else {
    throw Exception("Incompatible colorspace, can only hand YUV422_PLANAR images");
  }
}


void
JpegWriter::write()
{
  if ( buffer == NULL ) {
    throw Exception("JpegWriter::read() error: buffer == NULL");
  }

  if ((outfile = fopen(filename, "wb")) == NULL) {
    Exception e("Cannot open JPEG file for writing", errno);
    e.append("File %s could not be opened", filename);
    throw e;
  }

  int row_stride;
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr         jerr;

  cinfo.err = jpeg_std_error( &jerr );
  jpeg_create_compress( &cinfo );
  jpeg_stdio_dest( &cinfo, outfile );

  cinfo.image_width  = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, true /* limit to baseline-JPEG values */);

  jpeg_start_compress( &cinfo, true );
  row_stride = cinfo.image_width * cinfo.input_components;

  row_buffer = (unsigned char *)malloc( row_stride );

  while ( cinfo.next_scanline < cinfo.image_height ) {
    convert_line_yuv422planar_to_rgb( buffer, row_buffer,
				      cinfo.image_width, cinfo.image_height,
				      cinfo.next_scanline, 0 );
    jpeg_write_scanlines( &cinfo, &row_buffer, 1 );
  }

  free(row_buffer);

  jpeg_finish_compress( &cinfo );

  jpeg_destroy_compress( &cinfo );
  fclose( outfile );

}

} // end namespace firevision
