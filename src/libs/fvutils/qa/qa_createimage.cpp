/***************************************************************************
 *  qa_createimage.cpp - Simple test image creator
 *
 *  Created: Thu Mar 17 22:41:55 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

/***************************************************************************
 *  qa_createimage.cpp - Create simple test image for debayering
 *
 *  Created: Thu Mar 17 22:41:55 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/writers/fvraw.h>

#include <cstdlib>
#include <cstring>

using namespace firevision;

int
main(int argc, char **argv)
{
  unsigned char *buf = (unsigned char *)malloc(640 * 480);
  memset(buf, 0, 640*480);
  unsigned char *b = buf;

  for (unsigned int h = 0; h < 480; h += 2) {
    for (unsigned int w = 0; w < 640; w += 2) {
      *b++ = 255;
      ++b;
    }
    for (unsigned int w = 0; w < 640; w += 2) {
      ++b;
      *b++ = 255;
    }
  }

  FvRawWriter w("test.raw", 640, 480, MONO8, buf);
  w.write();

  return 0;
}

/// @endcond
