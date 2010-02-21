
/***************************************************************************
 *  qa_bb_remote.cpp - BlackBoard remote access QA
 *
 *  Created: Mon Mar 03 17:31:18 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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
#include <interfaces/TestInterface.h>

#include <signal.h>
#include <unistd.h>

using namespace fawkes;

bool quit = false;

void
signal_handler(int signum)
{
  quit = true;
}


int
main(int argc, char **argv)
{
  signal(SIGINT, signal_handler);

  BlackBoard *rbb = new RemoteBlackBoard("localhost", 1910);
  TestInterface *ti = rbb->open_for_writing<TestInterface>("BBLoggerTest");

  int i = 0;
  while (! quit) {
    ti->set_test_int(++i);
    ti->write();
    usleep(100000);
  }

  rbb->close(ti);
  sleep(2);
  delete rbb;
}


/// @endcond
