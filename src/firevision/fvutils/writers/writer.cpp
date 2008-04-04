
/***************************************************************************
 *  writer.cpp - Writer interface
 *
 *  Generated: Tue Mar 27 17:24:55 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#include <fvutils/writers/writer.h>

#include <core/exception.h>

#include <cstring>
#include <cstdlib>
#include <cstdio>

/** @class Writer <fvutils/writers/writer.h>
 * Interface to write images.
 * The writer interface defines the general API for image writers. These
 * writers are used to write images to files on your harddrive (like JPEGs,
 * PNGs etc.).
 *
 * @author Tim Niemueller
 */

/** @fn void Writer::write()
 * Write to file.
 */

/** @var Writer::filename
 * The complete filename.
 */

/** @var Writer::basename
 * The basename of the file.
 */
/** @var Writer::extension
 * The extension of the file.
 */
/** @var Writer::width
 * The width of the image.
 */
/** @var Writer::height
 * The height of the image.
 */
/** @var Writer::cspace
 * The colorspace of the image.
 */
/** @var Writer::buffer
 * The image-buffer.
 */

/** Constructor.
 * @param extension the file extension
 */
Writer::Writer(const char *extension)
{
  basename = 0;
  filename = 0;

  this->extension = 0;
  if (0 != extension) {
    this->extension = strdup(extension);
  }

  width = 0;
  height = 0;
  cspace = CS_UNKNOWN;
  buffer = 0;
}

/** Virtual empty destructor. */
Writer::~Writer()
{
  free(filename);
  free(basename);
  free(extension);
}

/** Set filename.
 * @param filename name of file to write to. This can either be the complete filename
 * (including) extension or the basename only in which case the extension is added.
 */
void
Writer::set_filename(const char *filename)
{
  free(this->filename);
  
  if ( 0 != strstr(filename, ".") ) {
    this->filename = strdup(filename);
  } else {
    free(this->basename);
    this->basename = strdup(filename);

    // re-generate complete filename
    if (0 == extension) {
      throw Exception("Extension not set");
    }

    asprintf(&(this->filename), "%s.%s", basename, extension);
  }
}

/** Set dimensions of image in pixels.
 * @param width width of image in pixels
 * @param height height of image in pixels.
 */
void
Writer::set_dimensions(unsigned int width, unsigned int height)
{
  this->width = width;
  this->height = height;
}

/** Set image buffer.
 * @param cspace color space of image
 * @param buffer buffer of image
 */
void
Writer::set_buffer(colorspace_t cspace, unsigned char *buffer)
{
  this->cspace = cspace;
  this->buffer = buffer;
}

/** Set the filename extension for file written by this writer.
 * @param extension the extension
 */
void
Writer::set_extension(const char *extension)
{
  free(this->extension);
  this->extension = strdup(extension);

  // re-generate complete filename
  free(this->filename);
  this->filename = (char *) malloc( strlen(basename) + strlen(extension) + 1 );
  strcpy(filename, basename);
  strcat(this->filename, ".");
  strcat(filename, extension);
}
