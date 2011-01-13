
/***************************************************************************
 *  qa_yuvconv.h - QA for YUV conversion
 *
 *  Created: Wed Jun 27 13:49:25 2007
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
#include <fvutils/color/yuv.h>

#include <fvutils/ipc/shm_image.h>

#include <iostream>

using namespace std;
using namespace firevision;

#define WIDTH  748
#define HEIGHT 572

int
main(int argc, char **argv)
{
  unsigned char *yuv422_packed = malloc_buffer(YUV422_PACKED, WIDTH, HEIGHT);

  //  unsigned char *yuv422_planar = malloc_buffer(YUV422_PLANAR, WIDTH, HEIGHT);
  SharedMemoryImageBuffer *shm = new SharedMemoryImageBuffer("fv_qa_yuvconv", YUV422_PLANAR, WIDTH, HEIGHT);
  unsigned char *yuv422_planar = shm->buffer();


  yuv422packed_to_yuv422planar(yuv422_packed, yuv422_planar, WIDTH, HEIGHT);

  delete shm;

  return 0;
}



/// @endcond
