
/***************************************************************************
 *  compressed_image_writer.cpp - Write image arbitrarily compressed with an
 *                                ImageCompressor
 *
 *  Generated: Sat Aug 12 14:03:08 2006
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
#include <core/exceptions/system.h>
#include <utils/system/console_colors.h>

#include <fvutils/writers/compressed.h>
#include <fvutils/compression/imagecompressor.h>

#include <cstdlib>
#include <cstring>
#include <cstdio>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CompressedImageWriter <fvutils/writers/compressed.h>
 * Writer for arbitrarily compressed images.
 * This class uses any image compressor to write compressed images to
 * a file.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param ic ImageCompressor to use for image compression
 */
CompressedImageWriter::CompressedImageWriter(ImageCompressor *ic)
{
  width = height = 0;
  filename = strdup("");
  cspace = CS_UNKNOWN;
  buffer = NULL;

  image_compressor = ic;
}


/** Destructor. */
CompressedImageWriter::~CompressedImageWriter()
{
  free(filename);
}


void
CompressedImageWriter::set_filename(const char *filename)
{
  free(this->filename);
  this->filename = strdup(filename);

  if ( image_compressor != NULL ) {
    image_compressor->set_filename( filename );
  }
}


void
CompressedImageWriter::set_dimensions(unsigned int width, unsigned int height)
{
  this->width  = width;
  this->height = height;
  if ( image_compressor != NULL ) {
    image_compressor->set_image_dimensions( width, height );
  }
}


void
CompressedImageWriter::set_buffer(colorspace_t cspace, unsigned char *buffer)
{
  this->cspace     = cspace;
  this->buffer     = buffer;
  if ( image_compressor != NULL ) {
    image_compressor->set_image_buffer( cspace, buffer );
  }
}


void
CompressedImageWriter::write()
{
  if ( image_compressor != NULL ) {
    if ( image_compressor->supports_compression_destination( ImageCompressor::COMP_DEST_FILE ) ) {
      image_compressor->set_compression_destination( ImageCompressor::COMP_DEST_FILE );
      image_compressor->compress();
    } else if ( image_compressor->supports_compression_destination( ImageCompressor::COMP_DEST_MEM ) ) {
      image_compressor->set_compression_destination( ImageCompressor::COMP_DEST_MEM );
      unsigned int comp_buffer_size = image_compressor->recommended_compressed_buffer_size();
      unsigned char *comp_buffer = (unsigned char *)malloc(comp_buffer_size);
      image_compressor->set_destination_buffer( comp_buffer, comp_buffer_size );
      image_compressor->compress();
      FILE *f = fopen(filename, "wb");
      if (fwrite(comp_buffer, image_compressor->compressed_size(), 1, f) != 1) {
	throw fawkes::FileWriteException(filename, "Failed to write data");
      }
      fclose(f);
      free(comp_buffer);
    } else {
      throw fawkes::Exception("Supplied ImageCompressor does neither support compressing "
			      " to file nor to a memory buffer. Cannot write.");
    }
  }
}


/** Set image compressor.
 * Use this method to change the used image compressor at runtime.
 * @param ic new image compressor.
 */
void
CompressedImageWriter::set_image_compressor(ImageCompressor *ic)
{
  image_compressor = ic;
  if ( ic != NULL ) {
    ic->set_filename( filename );
    ic->set_image_dimensions( width, height );
    ic->set_image_buffer(cspace, buffer);
  }
}

} // end namespace firevision
