
/***************************************************************************
 *  fvraw.h - FvRaw Reader
 *
 *  Generated: Sun Jun 05 01:22:35 2006 (watching Terminator 2)
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
#include <fvutils/readers/fvraw.h>
#include <fvutils/writers/fvraw.h>
#include <fvutils/color/colorspaces.h>

#include <cstdio>
#include <errno.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FvRawReader <fvutils/readers/fvraw.h>
 * FvRaw image reader implementation.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param filename filename to read from.
 */
FvRawReader::FvRawReader(const char *filename)
{
  opened = false;
  buffer = NULL;

  infile = fopen(filename, "r");

  if (infile == NULL) {
    throw Exception("Could not open file for reading");
  }

  if ( fread((char *)&header, sizeof(header), 1, infile) != 1 ) {
    throw Exception("Could not read header");
  } else {
    if (header.file_id != FvRawWriter::FILE_IDENTIFIER) {
      throw ("Invalid file identifier");
    } else {
      
      buffer_size = colorspace_buffer_size( header.colorspace, header.width, header.height );
      opened = true;
    }
  }
}


/** Destructor. */
FvRawReader::~FvRawReader()
{
  fclose( infile );
  opened = false;
}


void
FvRawReader::set_buffer(unsigned char *yuv422planar_buffer)
{
  buffer = yuv422planar_buffer;
}


colorspace_t
FvRawReader::colorspace()
{
  if ( opened ) {
    return header.colorspace;
  } else {
    return CS_UNKNOWN;
  }
}


unsigned int
FvRawReader::pixel_width()
{
  if ( opened ) {
    return header.width;
  } else {
    return 0;
  }
}


unsigned int
FvRawReader::pixel_height()
{
  if ( opened ) {
    return header.height;
  } else {
    return 0;
  }
}


void
FvRawReader::read()
{
  if ( buffer == NULL ) {
    throw Exception("Read failed: buffer == NULL");
  }
  if ( buffer_size == 0 ) {
    throw Exception("Read failed: buffer_size == 0");
  }

  if (fread(buffer, buffer_size, 1, infile) != 1) {
    throw Exception("Failed to read data", errno);
  }
}


/** Check if given file contains FvRaw image.
 * @param filename file to check
 * @return true if file contains FvRaw image, false otherwise
 */
bool
FvRawReader::is_FvRaw(const char *filename)
{
  FILE *f;
  f = fopen(filename, "r");
  if (f != NULL) {
    FvRawWriter::FvRawHeader header;
    if ( fread((char *)&header, sizeof(header), 1, f) == 1 ) {
      if (header.file_id == FvRawWriter::FILE_IDENTIFIER) {
	fclose(f);
	return true;
      }
    }
    fclose(f);
  }
  return false;
}

} // end namespace firevision
