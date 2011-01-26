
/***************************************************************************
 *  qa_fvcam.cpp - QA for Firewire camera
 *
 *  Created: Tue Apr 08 16:40:30 2008
 *  Copyright  2005-2008  Tim Niemueller [www.niemueller.de]
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

#include <cams/firewire.h>
#include <cams/factory.h>

#include <cstdio>

int
main(int argc, char **argv)
{
  Camera *cam;
  if ( argc > 1 ) {
    printf("Opening from camera argument string '%s'\n", argv[1]);
    cam = CameraFactory::instance(argv[1]);
  } else {
    printf("Opening plain Firewire camera\n");
    cam = new FirewireCamera();
  }
  cam->open();
  cam->start();
  cam->print_info();

  for (unsigned int i = 0; i < 100; ++i) {
    printf("Capturing\n");
    cam->capture();
    cam->dispose_buffer();
  }

  printf("Closing\n");
  cam->close();
  delete cam;

  return 0;
}

/// @endcond
