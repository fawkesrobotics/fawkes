
/***************************************************************************
 *  png.cpp - Implementation of a PNG writer
 *
 *  Generated: Thu Jun 02 15:23:56 2005
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
#include <fvutils/writers/png.h>
#include <fvutils/color/yuvrgb.h>

#include <cstdio>
#include <png.h>
#include <string.h>
#include <stdlib.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class PNGWriter <fvutils/writers/png.h>
 * PNG file writer.
 */

/** Constructor. */
PNGWriter::PNGWriter()
  : Writer("png")
{
}

/** Constructor.
 * @param filename filename
 * @param width width
 * @param height height
 */
PNGWriter::PNGWriter(const char *filename, unsigned int width, unsigned int height)
  : Writer("png")
{
  set_filename(filename);

  this->width    = width;
  this->height   = height;
}

/** Destructor. */
PNGWriter::~PNGWriter()
{
}

void
PNGWriter::set_buffer(colorspace_t cspace, unsigned char *buffer)
{
  if( cspace != BGR && cspace != RGB && cspace != YUV422_PLANAR) {
    throw Exception("Color space not supported, can only write YUV422_PLANAR images");
  }
  this->buffer = buffer; 
  colorspace_ = cspace;
}


void
PNGWriter::write()
{
  if ( (filename == 0) ||
       (width == 0) ||
       (height == 0) ) {
    throw Exception("PNGWriter::write(): Illegal data, width==0 || height == 0 || filename=\"\".");
  }

  FILE *fp = fopen(filename, "wb");
  if (!fp) {
    throw Exception("Could not open file for writing");
  }

  png_structp png_ptr = png_create_write_struct
    (PNG_LIBPNG_VER_STRING,(png_voidp)NULL,
     (png_error_ptr)NULL, (png_error_ptr)NULL);
  if (!png_ptr) {
    throw Exception("Could not create PNG write struct");
  }

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
    throw Exception("Could not create PNG info struct");
  }

  if (setjmp(png_jmpbuf(png_ptr))) {
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);
    png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
    throw Exception("Could not create setjmp");
  }

  // Use default io via fwrite
  png_init_io(png_ptr, fp);

  // Can be used to get informed about progress
  // png_set_write_status_fn(png_ptr, write_row_callback);

    png_set_IHDR(png_ptr, info_ptr, width, height,
		 8 /* bit per channel */,  PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
	       PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

  png_write_info(png_ptr, info_ptr);

  // png_byte == unsigned char, create one row, three bytes 
  //  png_byte row[width * 3];
  png_byte row[width*3]; 
  png_byte *row_p;
  unsigned char *yp, *up, *vp;
  unsigned char y1, y2, u = 0, v = 0;


  yp = buffer;
  up = YUV422_PLANAR_U_PLANE(buffer, width, height);
  vp = YUV422_PLANAR_V_PLANE(buffer, width, height);

  for (unsigned int i = 0; i < height; ++i) {
    if( colorspace_ == YUV422_PLANAR ) {
      // pack row
      row_p = row;
      for (unsigned int j = 0; j < (width / 2); ++j) {
	y1 = *yp++;
	y2 = *yp++;
	u  = *up++;
	v  = *vp++;
	pixel_yuv_to_rgb(y1, u, v, &row_p[0], &row_p[1], &row_p[2]);
	row_p += 3;
	pixel_yuv_to_rgb(y2, u, v, &row_p[0], &row_p[1], &row_p[2]);
	row_p += 3;
      }
      
      if ( (width % 2) == 1 ) {
	// odd number of columns, we have to take care of this
	// use last u,v values and new y value for this
	y1 = *yp++;
	pixel_yuv_to_rgb(y1, u, v, &row_p[0], &row_p[1], &row_p[2]);      
      }
    } else if (colorspace_ == BGR) {
      convert_line_bgr_rgb( (buffer + width*3*i), row,
			    width,  height ); 

    } else { // RGB
      memcpy(row, (buffer + width*3*i), width*3);
    }
    png_write_row(png_ptr, row);
  }

  png_write_end(png_ptr, info_ptr);
  png_destroy_write_struct(&png_ptr, &info_ptr);
  fclose(fp);

}

} // end namespace firevision
