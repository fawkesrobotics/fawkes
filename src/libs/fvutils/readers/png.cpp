
/***************************************************************************
 *  png.cpp - PNG Reader
 *
 *  Created: Thu Apr 03 12:56:56 2008
 *  Copyright  2005-2008  Tim Niemueller [www.niemueller.de]
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
#include <fvutils/readers/png.h>
#include <fvutils/color/rgbyuv.h>

#include <cstdio>
#include <cstdlib>
#include <png.h>
#include <cerrno>
#include <cstring>
#include <string>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/// @cond INTERNALS
class PNGReaderData
{
 public:
  FILE *infile;
  png_structp png_ptr;
  png_infop info_ptr;
  int number_passes;
  bool read;
};
/// @endcond

/** @class PNGReader <fvutils/readers/png.h>
 * PNG file reader.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param filename file to read
 */
PNGReader::PNGReader(const char *filename)
{
  opened = false;
  buffer = NULL;

  __d = setup_read(filename);

  opened = true;
}


PNGReaderData *
PNGReader::setup_read(const char *filename)
{
  PNGReaderData *d = new PNGReaderData();
  d->read = false;

  if ((d->infile = fopen(filename, "rb")) == NULL) {
    throw Exception("Cannot open PNG file %s: %s", filename, ::strerror(errno));
  }

  d->png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

  if (d->png_ptr == NULL) {
    fclose(d->infile);
    throw Exception("Could not create PNG read struct");
  }

  /* Allocate/initialize the memory for image information.  REQUIRED. */
  d->info_ptr = png_create_info_struct(d->png_ptr);
  if (d->info_ptr == NULL) {
    fclose(d->infile);
    png_destroy_read_struct(&d->png_ptr, (png_infopp)NULL, (png_infopp)NULL);
    throw Exception("Could not create PNG info struct");
  }

  /* Set error handling if you are using the setjmp/longjmp method (this is
   * the normal method of doing things with libpng).  REQUIRED unless you
   * set up your own error handlers in the png_create_read_struct() earlier.
   */
  if (setjmp(png_jmpbuf(d->png_ptr))) {
    std::string err(::strerror(errno));
    /* Free all of the memory associated with the png_ptr and info_ptr */
    png_destroy_read_struct(&d->png_ptr, &d->info_ptr, (png_infopp)NULL);
    fclose(d->infile);
    /* If we get here, we had a problem reading the file */
    throw Exception("Could not read PNG file %s: %s", filename, err.c_str());
  }

  /* Set up the input control if you are using standard C streams */
  png_init_io(d->png_ptr, d->infile);

  /* The call to png_read_info() gives us all of the information from the
   * PNG file before the first IDAT (image data chunk).  REQUIRED */
  png_read_info(d->png_ptr, d->info_ptr);
  
  /* tell libpng to strip 16 bit/color files down to 8 bits/color */
  png_set_strip_16(d->png_ptr);

  /* Strip alpha bytes from the input data without combining with the
   * background (not recommended). */
  png_set_strip_alpha(d->png_ptr);

  /* Extract multiple pixels with bit depths of 1, 2, and 4 from a single
   * byte into separate bytes (useful for paletted and grayscale images). */
  png_set_packing(d->png_ptr);

  png_byte color_type = png_get_color_type(d->png_ptr, d->info_ptr);

  /* Expand paletted colors into true RGB triplets */
  if (color_type == PNG_COLOR_TYPE_PALETTE)  png_set_palette_to_rgb(d->png_ptr);

  /* Expand grayscale images into true RGB triplets */
  if (color_type == PNG_COLOR_TYPE_GRAY)     png_set_gray_to_rgb(d->png_ptr);


  /* Tell libpng to handle the gamma conversion for you.  The final call
   * is a good guess for PC generated images, but it should be configurable
   * by the user at run time by the user.  It is strongly suggested that
   * your application support gamma correction. */
  int intent;
  double screen_gamma = 2.2;  /* A good guess for a PC monitors in a dimly lit room */
  if (png_get_sRGB(d->png_ptr, d->info_ptr, &intent)) {
    png_set_gamma(d->png_ptr, screen_gamma, 0.45455);
  } else {
    double image_gamma;
    if (png_get_gAMA(d->png_ptr, d->info_ptr, &image_gamma)) {
      png_set_gamma(d->png_ptr, screen_gamma, image_gamma);
    } else {
      png_set_gamma(d->png_ptr, screen_gamma, 0.45455);
    }
  }

  /* Turn on interlace handling.  REQUIRED if you are not using
   * png_read_image().  To see how to handle interlacing passes,
   * see the png_read_row() method below: */
  d->number_passes = png_set_interlace_handling(d->png_ptr);

  /* Optional call to gamma correct and add the background to the palette
   * and update info structure.  REQUIRED if you are expecting libpng to
   * update the palette for you (ie you selected such a transform above). */
  png_read_update_info(d->png_ptr, d->info_ptr);

  return d;
}

/** Destructor. */
PNGReader::~PNGReader()
{
  fclose( __d->infile );
  /* clean up after the read, and free any memory allocated - REQUIRED */
  png_destroy_read_struct(&__d->png_ptr, &__d->info_ptr, (png_infopp)NULL);

  delete __d;

  opened = false;
}


void
PNGReader::set_buffer(unsigned char *yuv422planar_buffer)
{
  buffer = yuv422planar_buffer;
}


colorspace_t
PNGReader::colorspace()
{
  return YUV422_PLANAR;
}


unsigned int
PNGReader::pixel_width()
{
  if ( opened ) {
    return png_get_image_width(__d->png_ptr, __d->info_ptr);
  } else {
    return 0;
  }
}


unsigned int
PNGReader::pixel_height()
{
  if ( opened ) {
    return png_get_image_height(__d->png_ptr, __d->info_ptr);
  } else {
    return 0;
  }
}


void
PNGReader::read()
{
  if ( buffer == NULL ) {
    throw Exception("PNGReader::read: buffer == NULL");
  }
  if ( __d->read ) {
    throw Exception("Can read PNG file only once.");
  }
  __d->read = true;

  png_bytep row_pointer;
  row_pointer = (png_bytep)png_malloc(__d->png_ptr, png_get_rowbytes(__d->png_ptr, __d->info_ptr));

  unsigned int lheight = pixel_height();
  unsigned int lwidth  = pixel_width();

  for (int pass = 0; pass < __d->number_passes; ++pass) {
    for (unsigned y = 0; y < lheight; ++y) {
      png_read_rows(__d->png_ptr, &row_pointer, (png_bytepp)NULL, 1);
      convert_line_rgb_to_yuv422planar( row_pointer, buffer, lwidth, lheight, 0, y );
    }
  }

  /* read rest of file, and get additional chunks in info_ptr - REQUIRED */
  png_read_end(__d->png_ptr, __d->info_ptr);
  png_free(__d->png_ptr, row_pointer);

}

} // end namespace firevision
