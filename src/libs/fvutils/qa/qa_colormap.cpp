
/***************************************************************************
 *  qa_colormap.cpp - QA for colormap
 *
 *  Created: Tue Apr 01 10:04:27 2008
 *  Copyright  2007-2008  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/colormap/yuvcm.h>
#include <fvutils/colormap/cmfile.h>
#include <fvutils/color/colorspaces.h>

#include <fvwidgets/image_display.h>
#include <core/exception.h>

#include <cstdlib>
#include <cstdio>

using namespace fawkes;
using namespace firevision;

#define BIGDEPTH 8

int
main(int argc, char **argv)
{

  const char *filename = "qatest.colormap";
  if ( argc > 1 ) {
    filename = argv[1];
  }

  printf("Creating simple one-plane colormap\n");
  YuvColormap *cm = new YuvColormap();

  for (unsigned int u = 100; u < 150; ++u) {
    for (unsigned int v = 100; v < 150; ++v) {
      cm->set(128, u, v, C_ORANGE);
    }
  }

  unsigned char *imgb = malloc_buffer(YUV422_PLANAR, cm->width() * 2, cm->height() * 2);
  cm->to_image(imgb);

  ImageDisplay *imgd = new ImageDisplay(cm->width() * 2, cm->height() * 2);
  imgd->show(imgb);

  imgd->loop_until_quit();

  delete cm;

  printf("Trying to create colormap with illegal depth, should throw an exception..");
  try {
    cm = new YuvColormap(3);
    printf(" Test failed, colormap was created\n");
    delete cm;
  } catch (Exception &e) {
    printf(" Test succeeded, caught exception\n");
  }

  printf("Trying colormap with depth of %u\n", BIGDEPTH);
  cm = new YuvColormap(BIGDEPTH);

  for (unsigned int d = 0; d < cm->depth(); ++d) {
    unsigned int y = 256 / cm->depth() * d;
    printf("d=%u   y=%u   u=[%u,%u]  v=[%u,%u]\n", d, y,
	   cm->depth() * d, cm->depth() * (d+1), cm->depth() * d, cm->depth() * (d+1));

    for (unsigned int u = cm->deepness() / cm->depth() * d; u < cm->deepness() / cm->depth() * (d+1); ++u) {
      for (unsigned int v = cm->deepness() / cm->depth() * d; v < cm->deepness() / cm->depth() * (d+1); ++v) {
	cm->set(y, u, v, C_ORANGE);
      }
    }

    cm->to_image(imgb, d);
    imgd->show(imgb);
    imgd->loop_until_quit();
  }

  printf("Saving colormap to a file\n");
  ColormapFile cmf;
  cmf.add_colormap(cm);
  cmf.write(filename);

  ColormapFile::ColormapBlockVector *blocks = cmf.colormap_blocks();
  printf("Written, created %zu blocks\n", blocks->size());
  delete blocks;
  delete cm;

  cmf.clear();

  printf("Reading colormap from file\n");
  cmf.read(filename);

  Colormap *tcm = cmf.get_colormap();
  if ( (cm = dynamic_cast<YuvColormap *>(tcm)) == 0 ) {
    printf("Error, did not get valid yuv colormap\n");
  } else {
    printf("Showing all %u colormap levels\n", cm->depth());
    for (unsigned int d = 0; d < cm->depth(); ++d) {
      cm->to_image(imgb, d);
      imgd->show(imgb);
      imgd->loop_until_quit();
    }
  }

  delete cm;

  unsigned int depth = 4, width = 128, height = 128;
  printf("Trying colormap with low resolution, choosing %dx%dx%d\n", depth, width, height);
  cm = new YuvColormap(depth, width, height);
  printf("YuvColormap dimensions: %dx%dx%d\n", cm->depth(), cm->width(), cm->height());
  ColormapFile cmfr(depth, width, height);
  delete cm;
  cmfr.write(filename);
  cmfr.clear();
  cmfr.read(filename);
  cm = dynamic_cast<YuvColormap *>(cmfr.get_colormap());
  printf("Read back colormap dimensions %dx%dx%d\n",
	 cm->depth(), cm->width(), cm->height());
  delete cm;

  //delete imgd;
  //free(imgb);
  return 0;
}

/// @endcond
