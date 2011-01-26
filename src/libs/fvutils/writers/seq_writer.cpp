
/***************************************************************************
 *  seq_writer.cpp - Writes sequences of images
 *
 *  Generated: Fri Jul 06 11:10:08 2007
 *  Copyright  2007  Daniel Beck
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

#include <fvutils/writers/seq_writer.h>
#include <core/exceptions/system.h>

#include <time.h>
#include <sys/time.h>

#include <cstring>
#include <cstdlib>
#include <cstdio>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SeqWriter <fvutils/writers/seq_writer.h>
 * Writes a sequence of images to disk.
 *
 * @author Daniel Beck
 */

/** Constructor.
 * @param writer the actual image writer
 */
SeqWriter::SeqWriter(Writer* writer)
{
  this->writer = writer;

  frame_number = 0;

  cspace = CS_UNKNOWN;

  filename = 0;
  img_path = 0;
}


/** Destructor.
 */
SeqWriter::~SeqWriter()
{
  delete writer;
  writer = 0;
  
  free(filename);
  free(img_path);
}

/** Set the path to where the images are stored.
 * @param img_path the image path
 */
void SeqWriter::set_path(const char* img_path)
{
  free(this->img_path);
  this->img_path = strdup(img_path);
  printf("SeqWriter: img path set to %s\n", this->img_path);
}

/** Set a (base-) filename.
 * If a filename is set the name of the files will look like this:
 * filename_index.ext .
 * @param filename the (base-) filename
 */
void SeqWriter::set_filename(const char* filename)
{
  free(this->filename);
  this->filename = strdup(filename);
}

/** Set the image dimensions.
 * @param width the width of the image
 * @param height the height of the image
 */
void SeqWriter::set_dimensions(unsigned int width, unsigned int height)
{
  writer->set_dimensions(width, height);
}

/** Set the colorspace of the image.
 * @param cspace the colospace
 */
void SeqWriter::set_colorspace(colorspace_t cspace)
{
  this->cspace = cspace;
}

/** Write a single image to disk.
 * A running number is added to the filename
 * @param buffer the image buffer that is written to disk
 */
void SeqWriter::write(unsigned char *buffer)
{
  ++frame_number;
  char* fn;

  time_t now = time(NULL);
  struct tm now_tm;
  struct timeval now_tv;

  gettimeofday(&now_tv, NULL);
  localtime_r(&now, &now_tm);

  char* timestring;
  if (asprintf(&timestring, "%04d%02d%02d_%02d%02d%02d_%06ld", now_tm.tm_year + 1900, 
	       now_tm.tm_mon + 1, now_tm.tm_mday, now_tm.tm_hour, now_tm.tm_min, 
	       now_tm.tm_sec, now_tv.tv_usec) == -1)
  {
    throw OutOfMemoryException("SeqWriter::write(): asprintf() failed (1)");
  }
  
  if (filename)
    {
      // filename: YYYYMMDD-hhmmss_uuuuuu_name_index.ext
      if (img_path)
      {
	if (asprintf(&fn, "%s/%s_%s-%04u", img_path, timestring, filename, frame_number) == -1)
	{
	  throw OutOfMemoryException("SeqWriter::write(): asprintf() failed (2)");
	}
      }
      else
      {
	if (asprintf(&fn, "%s_%s-%04u", timestring, filename, frame_number) == -1)
	{
	  throw OutOfMemoryException("SeqWriter::write(): asprintf() failed (2)");
	}
      }
    }	
  else
    {
      // filename: YYYYMMDD-hhmmss_uuuuuu_index.ext
      if (img_path)
      {
	if (asprintf(&fn, "%s/%s-%04u", img_path, timestring, frame_number) == -1)
	{
	  throw OutOfMemoryException("SeqWriter::write(): asprintf() failed (3)");
	}
      }
      else
      {
	if (asprintf(&fn, "%s-%04u", timestring, frame_number) == -1)
	{
	  throw OutOfMemoryException("SeqWriter::write(): asprintf() failed (4)");
	}
      }
    }

  writer->set_filename(fn);
  free(fn);

  try {
    writer->set_buffer(cspace, buffer);
    writer->write();
  } catch (Exception &e) {
    throw;
  }
}

} // end namespace firevision
