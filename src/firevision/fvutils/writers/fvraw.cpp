
/***************************************************************************
 *  fvraw.cpp - writer for FireVision raw files
 *
 *  Generated: Sat Mar 25 00:15:47 2006
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
#include <fvutils/writers/fvraw.h>

#include <string.h>
#include <stdlib.h>

#include <cstdio>
#include <cerrno>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** File identifier for FvRaw images. */
const unsigned int FvRawWriter::FILE_IDENTIFIER = 0x17559358; // 16

/** @class FvRawWriter <fvutils/writers/fvraw.h>
 * FvRaw Writer implementation.
 * This class allows for writing FvRaw images to a file.
 * @author Tim Niemueller
 */

/** Constructor. */
FvRawWriter::FvRawWriter()
  : Writer("raw")
{
  header.file_id = FILE_IDENTIFIER;
  header.width = 0;
  header.height = 0;
  header.colorspace = CS_UNKNOWN;

  buffer = NULL;
}


/** Constructor.
 * @param filename file name to write to
 * @param width width of image
 * @param height height of image
 */
FvRawWriter::FvRawWriter(const char *filename,
			 unsigned int width, unsigned int height)
  : Writer("raw")
{
  set_filename(filename);

  header.file_id    = FILE_IDENTIFIER;
  header.width      = width;
  header.height     = height;
  header.colorspace = CS_UNKNOWN;

  buffer = NULL;
}


/** Constructor.
 * @param filename file name to write to
 * @param width width of image
 * @param height height of image
 * @param colorspace colorspace
 * @param buffer buffer
 */
FvRawWriter::FvRawWriter(const char *filename,
			 unsigned int width, unsigned int height,
			 colorspace_t colorspace, unsigned char *buffer)
  : Writer("raw")
{
  set_filename(filename);

  header.file_id    = FILE_IDENTIFIER;
  header.width      = width;
  header.height     = height;
  header.colorspace = colorspace;  

  this->buffer = buffer;
}


/** Destructor. */
FvRawWriter::~FvRawWriter()
{
}


void
FvRawWriter::set_dimensions(unsigned int width, unsigned int height)
{
  header.width = width;
  header.height = height;
}


void
FvRawWriter::set_buffer(colorspace_t cspace, unsigned char *buffer)
{
  header.colorspace = cspace;
  this->buffer = buffer;
}


void
FvRawWriter::write()
{
  if ( strlen(filename) == 0 ) {
    throw Exception("Cannot write if no file name given");
  }
  if ( header.width == 0 ) {
    throw Exception("Cannot write if width = 0");
  }
  if ( header.height == 0 ) {
    throw Exception("Cannot write if height = 0");
  }
  if ( header.colorspace == CS_UNKNOWN ) {
    throw Exception("Cannot write if colorspace unknown");
  }
  if ( buffer == NULL ) {
    throw Exception("Cannot write if no buffer set");
  }

  FILE *imagefile=fopen(filename, "w");
  if( imagefile == NULL) {
    throw Exception("Cannot not open file for writing");
  }

  unsigned int buffer_size = colorspace_buffer_size(header.colorspace,
						    header.width,
						    header.height);

  if ( fwrite((const char *)&header, 1, sizeof(header), imagefile) != sizeof(header) ) {
    throw Exception("Cannot write header to file", errno);
    fclose(imagefile);
  }

  if ( fwrite((const char *)buffer, 1, buffer_size, imagefile) != buffer_size ) {
    throw Exception("Cannot write data to file", errno);
    fclose(imagefile);
  }

  fclose(imagefile);

}


/** Get write buffer.
 * @return write buffer
 */
unsigned char *
FvRawWriter::get_write_buffer()
{
  return buffer;
}

} // end namespace firevision
