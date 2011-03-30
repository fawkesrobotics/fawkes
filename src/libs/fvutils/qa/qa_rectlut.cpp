
/***************************************************************************
 *  qa_rectlut.h - QA for rectification LUT
 *
 *  Generated: Wed Oct 32 18:03:48 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/rectification/rectfile.h>
#include <fvutils/rectification/rectinfo_lut_block.h>

#include <list>
#include <cstdlib>
#include <iostream>
#include <cstdio>

using namespace std;
using namespace firevision;

#define WIDTH  640
#define HEIGHT 480

int
main(int argc, char **argv)
{
  srand(23423);

  const char *s = "qatest.rif";
  if ( argc > 1 ) {
    s = argv[1];
  }

  RectificationInfoFile *rif = new RectificationInfoFile(0xDEADBEEF, "No real camera");

  RectificationLutInfoBlock *rlib = new RectificationLutInfoBlock(WIDTH, HEIGHT,
								  FIREVISION_RECTINFO_CAMERA_MAIN);

  RectificationLutInfoBlock *rlib2 = new RectificationLutInfoBlock(WIDTH, HEIGHT,
								   FIREVISION_RECTINFO_CAMERA_LEFT);

  /* Random alternative, harder to read though
  for ( int i = 0; i < 10; ++i ) {
    uint16_t x, y, to_x, to_y;
    x=1+(uint16_t)(1.f * WIDTH * rand() / (RAND_MAX + 1.f));
    y=1+(uint16_t)(1.f * HEIGHT * rand() / (RAND_MAX + 1.f));
    to_x=1+(uint16_t)(1.f * WIDTH * rand() / (RAND_MAX + 1.f));
    to_y=1+(uint16_t)(1.f * HEIGHT * rand() / (RAND_MAX + 1.f));

    printf("Mapping (%u, %u) to (%u, %u)\n", x, y, to_x, to_y);
    rlib->set_mapping(x, y, to_x, to_y);
  }
  */

  for ( int i = 0; i < 10; ++i ) {
    uint16_t x = i, y = i, to_x = i * 2, to_y = i * 2;
    printf("Mapping (%u, %u) to (%u, %u)\n", x, y, to_x, to_y);
    rlib->set_mapping(x, y, to_x, to_y);
  }

  for ( int i = 10; i < 20; ++i ) {
    uint16_t x = i, y = i, to_x = i * 2, to_y = i * 2;
    printf("Mapping2 (%u, %u) to (%u, %u)\n", x, y, to_x, to_y);
    rlib2->set_mapping(x, y, to_x, to_y);
  }

  rif->add_rectinfo_block(rlib);
  rif->add_rectinfo_block(rlib2);

  RectificationInfoFile::RectInfoBlockVector *blocks = rif->rectinfo_blocks();

  for (RectificationInfoFile::RectInfoBlockVector::iterator i = blocks->begin(); i != blocks->end(); ++i) {
    RectificationLutInfoBlock *rlib = dynamic_cast<RectificationLutInfoBlock *>(*i);
    if ( rlib == NULL ) {
      printf("Got rectification info block of unknown type");
      continue;
    }

    printf("LUT:  type: %u  camera: %u  size: %zu\n",
	   rlib->type(), rlib->camera(), rlib->block_size());

    cout << "Looking for non-zero mappings" << endl;
    uint16_t x, y, to_x, to_y;
    for ( y = 0; y < HEIGHT; ++y) {
      for ( x = 0; x < WIDTH; ++x) {
	// Use evil (slow) method here, it's just for the QA...
	rlib->mapping(x, y, &to_x, &to_y);
	if ( (to_x != 0) || (to_y != 0) ) {
	  printf("(%u, %u) maps to (%u, %u)\n", x, y, to_x, to_y);
	}
      }
    }
  }

  delete blocks;

  cout << "Writing to " << s << endl;
  rif->write(s);

  rif->clear();

  cout << "Reading from " << s << endl;
  rif->read(s);

  blocks = rif->rectinfo_blocks();

  for (RectificationInfoFile::RectInfoBlockVector::iterator i = blocks->begin(); i != blocks->end(); ++i) {
    RectificationLutInfoBlock *rlib = dynamic_cast<RectificationLutInfoBlock *>(*i);
    if ( rlib == NULL ) {
      printf("Got rectification info block of unknown type");
      continue;

    }

    printf("LUT:  type: %u  camera: %u  size: %zu\n",
	   rlib->type(), rlib->camera(), rlib->block_size());

    cout << "Looking for non-zero mappings" << endl;
    uint16_t x, y, to_x, to_y;
    for ( y = 0; y < HEIGHT; ++y) {
      for ( x = 0; x < WIDTH; ++x) {
	// Use evil (slow) method here, it's just for the QA...
	rlib->mapping(x, y, &to_x, &to_y);
	if ( (to_x != 0) || (to_y != 0) ) {
	  printf("(%u, %u) maps to (%u, %u)\n", x, y, to_x, to_y);
	}
      }
    }
  }

  delete blocks;

  delete rif;
  return 0;
}



/// @endcond
