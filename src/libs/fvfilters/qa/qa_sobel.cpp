
/***************************************************************************
 *  qa_sobel.h - QA for Sobel filter
 *
 *  Generated: Mon May 21 17:42:25 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

/// @cond QA

#include <fvutils/adapters/iplimage.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/readers/jpeg.h>
#include <fvutils/draw/drawer.h>
#include <fvfilters/sobel.h>
#include <fvwidgets/image_display.h>
#include <utils/system/argparser.h>

#include <cstdlib>
#include <cstdio>
#include <cstring>

using namespace fawkes;
using namespace firevision;

int
main(int argc, char **argv)
{
  ArgumentParser* argp = new ArgumentParser( argc, argv, "h:f:c:" );

  if (argp->has_arg( "f" ))
    // read image from file
  {
    const char *image_file = argp->arg( "f" );
    
    JpegReader *reader = new JpegReader(image_file);
    unsigned char *buffer = malloc_buffer(YUV422_PLANAR,
					  reader->pixel_width(), reader->pixel_height());
    
    reader->set_buffer(buffer);
    reader->read();
    
    unsigned char *sobeled = malloc_buffer(YUV422_PLANAR,
					  reader->pixel_width(), reader->pixel_height());
    memset(sobeled + reader->pixel_width() * reader->pixel_height(), 128,
           reader->pixel_width() * reader->pixel_height());

    ROI *roi = ROI::full_image(reader->pixel_width(), reader->pixel_height());


    FilterSobel *sobelf = new FilterSobel();
    sobelf->set_src_buffer(buffer, roi, ORI_DEG_135);
    sobelf->set_dst_buffer(sobeled, roi);
    sobelf->apply();
    
    ImageDisplay *display = new ImageDisplay(reader->pixel_width(), reader->pixel_height());
    display->show(sobeled);
    display->loop_until_quit();
    
    delete display;
    
    delete roi;
    free(buffer);
    free(sobeled);
    delete sobelf;
    delete reader;
  }

  else
  {
    printf("Usage: %s -f <Image file as JPEG>\n", argv[0]);
    exit(-1);
  }

  delete argp;
}

/// @endcond
