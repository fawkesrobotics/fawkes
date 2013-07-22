
/***************************************************************************
 *  bb2info.cpp - Print BB2 to stdout
 *
 *  Created: Fri Nov 02 19:03:00 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

using namespace std;
using namespace firevision;

int
main(int argc, char **argv)
{

  CameraArgumentParser *cap = new CameraArgumentParser("bumblebee2:Bumblebee2 BB2-03S2C");

  Bumblebee2Camera *bb2 = new Bumblebee2Camera(cap);
  bb2->open();
  bb2->print_info();
  bb2->close();

  delete bb2;
  delete cap;

  return 0;
}

