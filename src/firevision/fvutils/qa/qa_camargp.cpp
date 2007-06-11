
/***************************************************************************
 *  qa_camargp.h - QA for camera argument parser
 *
 *  Generated: Wed Apr 11 16:02:33 2007
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

/// @cond QA

#include <fvutils/system/camargp.h>

#include <iostream>

using namespace std;

int
main(int argc, char **argv)
{
  const char *s = "firewire:funny ID:mode=xy:test=test2:blub";
  if ( argc > 1 ) {
    s = argv[1];
  }

  CameraArgumentParser *argp = new CameraArgumentParser(s);

  cout << "Camera Type: " << argp->cam_type() << endl;
  cout << "Camera ID:   " << argp->cam_id() << endl;

  map<string, string> values = argp->parameters();
  map<string, string>::iterator i;
  for (i = values.begin(); i != values.end(); ++i) {
    cout << "values[" << (*i).first << "] = " << (*i).second << endl;
  }

  vector<string> args = argp->arguments();
  vector<string>::iterator j;
  for (j = args.begin(); j != args.end(); ++j) {
    cout << "arg: " << (*j) << endl;
  }

  delete argp;

  return 0;
}



/// @endcond
