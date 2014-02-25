
/***************************************************************************
 *  qa_jpegbm.h - QA for benchmarking jpeg compression
 *
 *  Created: Fri Jul 20 13:22:51 2007
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

/// @cond QA

#include <fvutils/color/colorspaces.h>
#include <fvutils/compression/jpeg_compressor.h>

#include <utils/time/tracker.h>
#include <iostream>
#include <cstdlib>

using namespace std;
using namespace fawkes;
using namespace firevision;

#define IMAGE_WIDTH   500
#define IMAGE_HEIGHT  500

#define NUM_CYCLES 100

// ~ 500 KB should be enough
#define DEST_BUF_SIZE 500000

int
main(int argc, char **argv)
{

  unsigned char *yuv422planar = malloc_buffer(YUV422_PLANAR, IMAGE_WIDTH, IMAGE_HEIGHT);
  unsigned char *compressed = (unsigned char *)malloc(DEST_BUF_SIZE);

  JpegImageCompressor *jpeg = new JpegImageCompressor(JpegImageCompressor::JPEG_CI_MMAL, 40,
						      JpegImageCompressor::JPEG_CS_RGB);
  //jpeg->set_filename("test.jpg");
  jpeg->set_image_dimensions(IMAGE_WIDTH, IMAGE_HEIGHT);
  jpeg->set_image_buffer(YUV422_PLANAR, yuv422planar);
  jpeg->set_destination_buffer(compressed, DEST_BUF_SIZE);
  jpeg->set_compression_destination(ImageCompressor::COMP_DEST_MEM);

  TimeTracker *tracker = new TimeTracker(true);

  for ( unsigned int i = 0; i < NUM_CYCLES; ++i) {
    printf("Compress %u\n", i);
    jpeg->compress();
    printf("Ping\n");
    tracker->ping(0);
    printf("Done %u with %zu bytes\n", i, jpeg->compressed_size());
  }

  tracker->print_to_stdout();

  delete tracker;
  delete jpeg;
  free(compressed);
  free(yuv422planar);

  return 0;
}



/// @endcond
