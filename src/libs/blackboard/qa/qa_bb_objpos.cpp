
/***************************************************************************
 *  qa_bb_objpos.h - BlackBoard QA: open a few ObjectPositionInterfaces
 *
 *  Created: Mon Jan 12 13:46:16 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <blackboard/remote.h>
#include <blackboard/exceptions.h>
#include <blackboard/bbconfig.h>

#include <interfaces/ObjectPositionInterface.h>

#include <core/exceptions/system.h>
#include <utils/logging/liblogger.h>

#include <signal.h>
#include <cstdlib>

#include <iostream>
#include <vector>

using namespace std;
using namespace fawkes;

bool quit = false;

void handle_signal(int signum)
{
  quit = true;
}

int
main(int argc, char **argv)
{
  signal(SIGINT, handle_signal);

  LibLogger::init();
  BlackBoard *bb = new RemoteBlackBoard("localhost", 1910);

  std::list<ObjectPositionInterface *> interfaces;

  cout << "Opening interfaces" << endl;
  for (int i = 1; i <= 10; ++i) {
    char tmp[100];
    sprintf(tmp, "Obstacle %i", i);
    printf("   %s\n", tmp);
    ObjectPositionInterface *iface = bb->open_for_writing<ObjectPositionInterface>(tmp);
    interfaces.push_back(iface);
  }

  while ( ! quit) sleep(1);

  for (std::list<ObjectPositionInterface *>::iterator i = interfaces.begin(); i != interfaces.end(); ++i) {
    bb->close(*i);
  }

  delete bb;
  LibLogger::finalize();
}


/// @endcond
