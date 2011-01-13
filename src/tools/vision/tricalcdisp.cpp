
/***************************************************************************
 *  calcdisp.cpp - Calculate disparities for the given images
 *
 *  Created: Mon Oct 08 13:42:01 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/writers/jpeg.h>
#include <fvutils/readers/fvraw.h>
#include <fvutils/color/conversions.h>

#include <fvstereo/triclops.h>

#include <list>
#include <string>

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <cstdlib>

using namespace std;
using namespace fawkes;
using namespace firevision;

int
main(int argc, char **argv)
{

  if ( argc < 3 ) {
    printf("Usage: %s <image> <triclops_context>\n", argv[0]);
    exit(-1);
  }

  const char *file = argv[1];
  const char *context_file = argv[2];

  char *outfile;
  asprintf(&outfile, "%s.jpg", file);

  JpegWriter *jpeg = new JpegWriter(outfile);

  try {
    FvRawReader *fvraw = new FvRawReader(file);

    unsigned int width = fvraw->pixel_width();
    unsigned int height = fvraw->pixel_height();

    if ( fvraw->colorspace() != RAW16 ) {
      printf("Can only operate on RAW16 images!\n");
      delete jpeg;
      delete fvraw;
      return -1;
    }

    printf("Calculating disparity for %s to %s\n", file, outfile);
    printf("Using Triclops context file %s\n", context_file);

    unsigned char *raw16 = malloc_buffer(RAW16, width, height);
    unsigned char *yuv422_planar = malloc_buffer(YUV422_PLANAR, width, height);

    TriclopsStereoProcessor *tsp = new TriclopsStereoProcessor(width, height, context_file);

    fvraw->set_buffer(raw16);
    fvraw->read();

    tsp->set_raw_buffer(raw16);

    tsp->preprocess_stereo();
    tsp->calculate_disparity();

    memcpy(yuv422_planar, tsp->disparity_buffer(), width * height);
    memset(yuv422_planar + width * height, 128, width * height);

    jpeg->set_buffer(YUV422_PLANAR, yuv422_planar);
    jpeg->set_dimensions(width, height);
    jpeg->write();

    delete jpeg;
    delete fvraw;
    free(raw16);
    free(yuv422_planar);
    delete tsp;
    free(outfile);

  } catch (Exception &e) {
    e.print_trace();
    throw;
  }
}
