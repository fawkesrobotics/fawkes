
/***************************************************************************
 *  converter.cpp - Convert between file formats supported by Firevision
 *
 *  Created: Tue Jul 05 14:34:21 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <iostream>

#include <string.h>
#include <stdlib.h>

#include <fvutils/writers/fvraw.h>
#include <fvutils/writers/jpeg.h>
#include <fvutils/writers/png.h>
#include <fvutils/writers/pnm.h>

#include <fvutils/readers/fvraw.h>
#include <fvutils/readers/jpeg.h>

#include <fvutils/color/conversions.h>

#define BUFFER_SIZE 5000000

using namespace std;

int main(int argc, char** argv)
{
  if (3 != argc)
    {
      cout << "Usage: " << argv[0] << " infile outfile" << endl;
      return 0;
    }

  char* fn_in = argv[1];
  char* fn_out = argv[2];

  char* fn_in_copy = strdup(fn_in);
  char* fn_out_copy = strdup(fn_out);
  
  cout << "Input file: " << fn_in_copy << endl;
  cout << "Output file: " << fn_out_copy << endl;
  
  // strip off extension
  char* t = strtok(fn_in_copy, ".");
  if (NULL == t)
    {
      cout << "invalid filename" << endl;
      return 0;
    }

  char* ext_in;
  while(NULL != t)
    {
      ext_in = t;
      t = strtok(NULL, ".");
    }
  
  t = strtok(fn_out_copy, ".");
  if (NULL == t)
    {
      cout << "invalid filename" << endl;
      return 0;
    }

  char* ext_out;
  while(NULL != t)
    {
      ext_out = t;
      t = strtok(NULL, ".");
    }

  Reader* reader = NULL;
  Writer* writer = NULL;

  // FvRaw
  if ( 0 == strcmp(ext_in, "raw") )
    {
      cout << "Format of file " << fn_in << " is FvRaw" << endl;
      reader = new FvRawReader(fn_in);
    }
  // JPEG
  else if ( 0 == strcmp(ext_in, "jpg") || 0 == strcmp(ext_in, "jpeg") )
    {
      cout << "Format of file " << fn_in << " is Jpeg" << endl;
      reader = new JpegReader(fn_in);
    }
  else
    {
      cout << "Unknown input file format" << endl;
      exit(-1);
    }
  
  unsigned char* buffer = (unsigned char*) malloc(BUFFER_SIZE);
  reader->set_buffer(buffer);
  reader->read();

  unsigned char *tmpbuf = malloc_buffer(YUV422_PLANAR, reader->pixel_width(), reader->pixel_height());
  convert(reader->colorspace(), YUV422_PLANAR, buffer, tmpbuf,
	  reader->pixel_width(), reader->pixel_height());

  // FvRaw
  if ( 0 == strcmp(ext_out, "raw") )
    {
      cout << "Format of file " << fn_out << " is FvRaw" << endl;
      writer = new FvRawWriter();
    }
  // JPEG
  else if ( 0 == strcmp(ext_out, "jpeg") || 0 == strcmp(ext_out, "jpg") )
    { 
      cout << "Format of file " << fn_out << " is Jpeg" << endl;
      writer = new JpegWriter();
    }
  // PNG
  else if ( 0 == strcmp(ext_out, "png") )
    {
      cout << "Format of file " << fn_out << " is PNG" << endl;
      writer = new PNGWriter();
    }
  // PNM
  else if ( 0 == strcmp(ext_out, "pnm") )
    {
      cout << "Format of file " << fn_out << " is PNM" << endl;
      writer = new PNMWriter(PNM_PPM);
    }
  else
    {
      cout << "Unknown output file format" << endl;
      exit(-2);
    }

  writer->set_filename(fn_out);
  writer->set_dimensions(reader->pixel_width(), reader->pixel_height());
  writer->set_buffer(YUV422_PLANAR, tmpbuf);
  writer->write();

  free(fn_in_copy);
  free(fn_out_copy);

  delete reader;
  delete writer;

  free(buffer);
  free(tmpbuf);
  
  return 0;
}
