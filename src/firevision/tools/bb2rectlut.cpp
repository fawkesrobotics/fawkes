
/***************************************************************************
 *  bb2rectlut.cpp - Generate BB2 rectification LUT from Triclops context
 *
 *  Created: Mon Oct 29 19:04:28 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <cams/bumblebee2.h>
#include <fvutils/system/camargp.h>
#include <stereo/triclops.h>

using namespace std;

int
main(int argc, char **argv)
{

  if ( argc < 2 ) {
    printf("Usage: %s <lut_file>\n", argv[0]);
    exit(-1);
  }

  const char *lut_file = argv[1];
  if ( access(lut_file, F_OK) == 0) {
    fprintf(stderr, "File with name %s exists, delete manually and retry. Aborting.\n", lut_file);
    return -1;
  }
  if ( access(lut_file, W_OK) != 0) {
    // ENOENT is ok, we would have access, but there is no file, yet
    if ( errno != ENOENT ) {
      fprintf(stderr, "Cannot write to file %s, permission problem?\n", lut_file);
      return -2;
    }
  }

  CameraArgumentParser *cap = new CameraArgumentParser("bumblebee2:Bumblebee2 BB2-03S2C");

  Bumblebee2Camera *bb2 = new Bumblebee2Camera(cap);
  bb2->open();

  TriclopsStereoProcessor *triclops = new TriclopsStereoProcessor(bb2);
  triclops->generate_rectification_lut(lut_file);
  delete triclops;

  bb2->close();

  delete bb2;
  delete cap;

  return 0;
}
