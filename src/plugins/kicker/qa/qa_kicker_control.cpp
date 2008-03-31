
/***************************************************************************
 *  qa_kicker_control.cpp - Kicker Control QA Application
 *
 *  Generated: Tue May 14 13:36:37 2007
 *  Copyright  2007  Daniel Beck
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

#include <plugins/kicker/kicker_control.h>

#include <iostream>
#include <signal.h>
#include <unistd.h>

using namespace std;

bool quit = false;

void
signal_handler(int signum)
{
  quit = true;
}

int
main(int argc, char* argv[])
{
  signal(SIGINT, signal_handler);

  KickerControl kicker_control;
  kicker_control.open();

  cout << endl;
  cout << "set_intensity()" << endl;
  kicker_control.set_intensity(0xFF);
  sleep(1);
  cout << endl;

  cout << "kick_right()" << endl;
  kicker_control.kick_right();
  sleep(2);
  cout << endl;

  cout << "kick_center()" << endl;
  kicker_control.kick_center();
  sleep(2);
  cout << endl;

  cout << "kick_left()" << endl;
  kicker_control.kick_left();
  sleep(2);
}



/// @endcond
