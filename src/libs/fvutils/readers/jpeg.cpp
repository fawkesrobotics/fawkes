
/***************************************************************************
 *  jpeg.cpp - JPEG Reader
 *
 *  Generated: Sun Jun 04 23:18:06 2006 (watching Terminator 2)
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

#include <core/exception.h>
#include <fvutils/readers/jpeg.h>
#include <fvutils/color/rgbyuv.h>

#include <cstdio>
#include <cstdlib>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class JpegReader <fvutils/readers/jpeg.h>
 * JPEG file reader.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param filename file to read
 */
JpegReader::JpegReader(const char *filename)
{
  opened = false;
  buffer = NULL;

  if ((infile = fopen(filename, "rb")) == NULL) {
    throw Exception("Cannot open JPEG file");
  }

  cinfo.err = jpeg_std_error( &jerr );
  jpeg_create_decompress( &cinfo );
  jpeg_stdio_src( &cinfo, infile );

  jpeg_read_header( &cinfo, true );
  jpeg_calc_output_dimensions( &cinfo );

  /*
  cout << "Read JPEG header, image info:" << endl
       << "  width:   " << cinfo.output_width << endl
       << "  height:  " << cinfo.output_height << endl;
  */

  opened = true;
}


/** Destructor. */
JpegReader::~JpegReader()
{
  jpeg_destroy_decompress( &cinfo );
  fclose( infile );
  opened = false;
}


void
JpegReader::set_buffer(unsigned char *yuv422planar_buffer)
{
  buffer = yuv422planar_buffer;
}


colorspace_t
JpegReader::colorspace()
{
  return YUV422_PLANAR;
}


unsigned int
JpegReader::pixel_width()
{
  if ( opened ) {
    return cinfo.output_width;
  } else {
    return 0;
  }
}


unsigned int
JpegReader::pixel_height()
{
  if ( opened ) {
    return cinfo.output_height;
  } else {
    return 0;
  }
}


void
JpegReader::read()
{
  if ( buffer == NULL ) {
    throw Exception("JpegReader::read: buffer == NULL");
  }

  jpeg_start_decompress( &cinfo );
  row_stride = cinfo.output_width * cinfo.output_components;

  row_buffer = (unsigned char *)malloc( row_stride );

  while ( cinfo.output_scanline < cinfo.output_height ) {
    jpeg_read_scanlines( &cinfo, &row_buffer, 1 );
    convert_line_rgb_to_yuv422planar( row_buffer, buffer,
				      cinfo.output_width, cinfo.output_height,
				      0, cinfo.output_scanline - 1 );
  }

  free( row_buffer );
  jpeg_finish_decompress( &cinfo );

}

} // end namespace firevision
