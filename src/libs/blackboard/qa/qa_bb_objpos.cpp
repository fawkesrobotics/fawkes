
/***************************************************************************
 *  qa_bb_objpos.h - BlackBoard QA: open a few ObjectPositionInterfaces
 *
 *  Created: Mon Jan 12 13:46:16 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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
#include <blackboard/local.h>
#include <blackboard/exceptions.h>
#include <blackboard/bbconfig.h>
#include <netcomm/fawkes/server_thread.h>

#include <interfaces/ObjectPositionInterface.h>

#include <core/exceptions/system.h>
#include <logging/liblogger.h>
#include <utils/time/tracker.h>

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
  //BlackBoard *bb = new RemoteBlackBoard("localhost", 1910);
  LocalBlackBoard *lbb = new LocalBlackBoard(BLACKBOARD_MEMSIZE);
  BlackBoard *bb = lbb;
  FawkesNetworkServerThread *netthread = new FawkesNetworkServerThread(1910);
  netthread->start();
  lbb->start_nethandler(netthread);

  std::list<ObjectPositionInterface *> interfaces;

  cout << "Opening interfaces" << endl;
  for (int i = 1; i <= 15; ++i) {
    char tmp[100];
    sprintf(tmp, "legtracker Leg %i", i);
    printf("   %s\n", tmp);
    ObjectPositionInterface *iface = bb->open_for_writing<ObjectPositionInterface>(tmp);
    interfaces.push_back(iface);
  }

  srand(time(NULL));

  TimeTracker tt;
  unsigned int ttc_write = tt.add_class("Write");

  int u = 0;
  while ( ! quit) {
    for (std::list<ObjectPositionInterface *>::iterator i = interfaces.begin(); i != interfaces.end(); ++i) {
      int r = rand() % 1000000;
      (*i)->set_world_x((float)r);
      (*i)->set_world_y((float)r+1);
      (*i)->set_world_z((float)r+2);
      tt.ping_start(ttc_write);
      (*i)->write();
      tt.ping_end(ttc_write);
    }
    if ( ++u > 20000 ) {
      tt.print_to_stdout();
      tt.reset();
      u = 0;
    }
    //sleep(1);
  }

  for (std::list<ObjectPositionInterface *>::iterator i = interfaces.begin(); i != interfaces.end(); ++i) {
    bb->close(*i);
  }

  delete bb;
  LibLogger::finalize();
}


/// @endcond
