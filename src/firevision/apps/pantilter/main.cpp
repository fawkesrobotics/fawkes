
/***************************************************************************
 *  main.cpp - RCSoftX pan-tilter
 *
 *  Created: Tue Apr 10 14:33:19 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: main.cpp 899 2008-04-10 11:36:58Z tim $
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

/// @cond RCSoftX

#include <iostream>

#include "bbclient.h"
#include <utils/system/console_colors.h>

using namespace std;
using namespace fawkes;

int
main( int argc, char **argv )
{

  cout << cblue << "FirevisionPanTilter" << cnormal << ": Running BBClient" << endl;

  FirevisionPanTilterBBClient *bbclient = new FirevisionPanTilterBBClient(argc, argv);
  bbclient->Main();
  delete bbclient;

}

/// @endcond
