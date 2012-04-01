
/***************************************************************************
 *  bb2gettric.cpp - Get Triclops context from BB2 camera
 *
 *  Created: Mon Oct 08 14:12:39 2007
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

#include <fvcams/bumblebee2.h>
#include <fvutils/system/camargp.h>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>

using namespace std;
using namespace firevision;

int
main(int argc, char **argv)
{

  if ( argc < 2 ) {
    printf("Usage: %s <context_file>\n", argv[0]);
    exit(-1);
  }

  const char *context_file = argv[1];
  if ( access(context_file, F_OK) == 0) {
    fprintf(stderr, "File with name %s exists, delete manually and retry. Aborting.\n", context_file);
    return -1;
  }
  if ( access(context_file, W_OK) != 0) {
    // ENOENT is ok, we would have access, but there is no file, yet
    if ( errno != ENOENT ) {
      fprintf(stderr, "Cannot write to file %s, permission problem?\n", context_file);
      return -2;
    }
  }

  CameraArgumentParser *cap = new CameraArgumentParser("bumblebee2:Bumblebee2 BB2-03S2C");

  Bumblebee2Camera *bb2 = new Bumblebee2Camera(cap);
  bb2->open();

  bb2->write_triclops_config_from_camera_to_file(context_file);
  bb2->close();
  delete bb2;
  delete cap;

  return 0;
}
