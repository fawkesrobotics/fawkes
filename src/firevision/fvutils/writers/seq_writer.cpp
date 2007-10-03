
/***************************************************************************
 *  seq_writer.cpp - Writes sequences of images
 *
 *  Generated: Fri Jul 06 11:10:08 2007
 *  Copyright  2007  Daniel Beck
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

#include <fvutils/writers/seq_writer.h>

#include <string.h>
#include <time.h>
#include <stdlib.h>

#include <cstdio>

/** @class SeqWriter <fvutils/writers/seq_writer.h>
 * Writes a sequence of images to disk.
 */

/** Constructor.
 * @param writer the actual image writer
 */
SeqWriter::SeqWriter(Writer* writer)
{
  this->writer = writer;

  frame_number = 0;

  cspace = CS_UNKNOWN;

  filename = "";
  img_path = "";
}


/** Destructor.
 */
SeqWriter::~SeqWriter()
{
  delete writer;
  writer = 0;
  
  if ("" != filename) {
    free(filename);
  }

  if ("" != img_path) {
    free(img_path);
  }
}

/** Set the path to where the images are stored.
 * @param img_path the image path
 */
void SeqWriter::set_path(const char* img_path)
{
  this->img_path = strdup(img_path);
}

/** Set a (base-) filename.
 * If a filename is set the name of the files will look like this:
 * filename_index.ext .
 * @param filename the (base-) filename
 */
void SeqWriter::set_filename(const char* filename)
{
  if (strcmp(this->filename, "") != 0)
    {
      free(this->filename);
    }
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
 * @param _cspace the colospace
 */
void SeqWriter::set_colorspace(colorspace_t _cspace)
{
  cspace = _cspace;
}

/** Write a single image to disk.
 * A running number is added to the filename
 * @param buffer the image buffer that is written to disk
 */
void SeqWriter::write(unsigned char *buffer)
{
  frame_number++;
  char* fn;

  time_t now = time(NULL);
  struct tm *t = localtime(&now);
  char timestring[30];
  strftime(timestring, 30, "%Y%m%d-%H%M%S", t);
  
  if (strcmp(filename, "") != 0)
    {
      // filename: YYYYMMDD-hhmmss_index.ext
      fn = (char*) malloc( strlen(img_path) + strlen(timestring) + 10 );
      if (strcmp(img_path, "") != 0)
	{
	  sprintf(fn, "%s/%s_%04u", img_path, timestring, frame_number);
	}
      else
	{
	  sprintf(fn, "%s_%04u", timestring, frame_number);
	}
    }
  else
    {
      // filename: YYYYMMDD-hhmmss_name_index.ext
      fn = (char*) malloc( strlen(img_path) + strlen(timestring) + strlen(filename) + 10 );
      if (strcmp(img_path, "") != 0)
	{
	  sprintf(fn, "%s/%s_%s_%04u", img_path, timestring, filename, frame_number);
	}
      else
	{
	  sprintf(fn, "%s_%s_%04u", timestring, filename, frame_number);
	}
    }

  writer->set_filename(fn);
  free(fn);

  try {
    writer->set_buffer(cspace, buffer);
    writer->write();
  } catch (Exception &e) {
    e.printTrace();
    throw;
  }
}

