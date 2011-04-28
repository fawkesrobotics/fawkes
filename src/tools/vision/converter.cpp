
/***************************************************************************
 *  converter.cpp - Convert between file formats supported by Firevision
 *
 *  Created: Tue Jul 05 14:34:21 2007
 *  Copyright  2007  Daniel Beck
 *             2008  Tim Niemueller [www.niemueller.de]
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

#include <fvcams/fileloader.h>
#include <fvutils/writers/fvraw.h>
#ifdef HAVE_LIBJPEG
#  include <fvutils/writers/jpeg.h>
#endif
#ifdef HAVE_LIBPNG
#  include <fvutils/writers/png.h>
#endif
#include <fvutils/writers/pnm.h>

#include <fvutils/readers/fvraw.h>
#include <fvutils/readers/jpeg.h>

#include <fvutils/color/conversions.h>
#include <utils/system/argparser.h>

#include <cstring>
#include <cstdlib>

using namespace fawkes;
using namespace firevision;

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-u -c colorspace -w width -h height] <infile> <outfile>\n\n"
         "  -u             Unformatted raw, you must supply -c, -w and -h\n"
	 "  -c colorspace  colorspace string\n"
	 "  -w width       width of image in pixels\n"
	 "  -h height      height of image in pixels\n",
	 program_name);
}


int
main(int argc, char** argv)
{
  ArgumentParser argp(argc, argv, "uw:h:c:");
  if ( argp.num_items() != 2 )
  {
    print_usage(argp.program_name());
    printf("\nInvalid number of files given\n\n");
    return -1;
  }

  const char *fn_in  = argp.items()[0];
  const char *fn_out = argp.items()[1];

  char* fn_out_copy = strdup(fn_out);
  
  printf("Input file:  %s\n"
	 "Output file: %s\n",
	 fn_in, fn_out);
  
  // strip off extension
  char *t = strtok(fn_out_copy, ".");
  if (NULL == t)
  {
    printf("invalid filename");
    return -2;
  }

  char* ext_out;
  while(NULL != t)
  {
    ext_out = t;
    t = strtok(NULL, ".");
  }

  FileLoader *fl = NULL;
  Writer* writer = NULL;

  if ( argp.has_arg("u") )
  {
    if (argp.has_arg("c") && argp.has_arg("w") && argp.has_arg("h"))
    {
      fl = new FileLoader(colorspace_by_name(argp.arg("c")), fn_in,
			  argp.parse_int("w"), argp.parse_int("h"));
      printf("Input image: %s, %lix%li\n", argp.arg("c"),
	     argp.parse_int("w"), argp.parse_int("h"));
    }
    else
    {
      printf("You have to supply all of -w, -h, -c when using -u.\n");
      return -3;
    }
  }
  else
  {
    fl = new FileLoader(fn_in);
  }

  fl->open();
  fl->start();

  unsigned char *tmpbuf = malloc_buffer(YUV422_PLANAR, fl->pixel_width(), fl->pixel_height());
  convert(fl->colorspace(), YUV422_PLANAR, fl->buffer(), tmpbuf,
	  fl->pixel_width(), fl->pixel_height());

  // FvRaw
  if ( 0 == strcmp(ext_out, "raw") )
  {
    printf("Format for out file %s is FvRaw\n", fn_out);
    writer = new FvRawWriter();
  }
#ifdef HAVE_LIBJPEG
  // JPEG
  else if ( 0 == strcmp(ext_out, "jpeg") || 0 == strcmp(ext_out, "jpg") )
  { 
    printf("Format for out file %s is Jpeg\n", fn_out);
    writer = new JpegWriter();
  }
#endif
#ifdef HAVE_LIBPNG
  // PNG
  else if ( 0 == strcmp(ext_out, "png") )
  {
    printf("Format for out file %s is PNG\n", fn_out);
    writer = new PNGWriter();
  }
#endif
  // PNM
  else if ( 0 == strcmp(ext_out, "pnm") )
  {
    printf("Format for out file %s is PNM\n", fn_out);
    writer = new PNMWriter(PNM_PPM);
  }
  else
  {
    printf("Unknown output file format\n");
    exit(-2);
  }

  writer->set_filename(fn_out);
  writer->set_dimensions(fl->pixel_width(), fl->pixel_height());
  writer->set_buffer(YUV422_PLANAR, tmpbuf);
  writer->write();

  free(fn_out_copy);

  delete fl;
  delete writer;

  free(tmpbuf);
  
  return 0;
}
