
/***************************************************************************
 *  qa_shmimg.h - QA for shared memory image
 *
 *  Created: Thu Oct 18 19:45:54 2007
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

#include <fvutils/ipc/shm_image.h>

#include <iostream>

using namespace std;
using namespace firevision;

int
main(int argc, char **argv)
{
  SharedMemoryImageBuffer *buf, *buf2;

  buf = new SharedMemoryImageBuffer("QA test image", YUV422_PLANAR, 100, 100);
  buf2 = new SharedMemoryImageBuffer("QA test image 2", YUV422_PLANAR, 100, 100);

  if ( buf->is_valid() ) {
    cout << "IS valid!" << endl;
  } else {
    cout << "Is NOT valid!" << endl;
  }

  sleep(100);

  delete buf;
  delete buf2;

  return 0;
}



/// @endcond
