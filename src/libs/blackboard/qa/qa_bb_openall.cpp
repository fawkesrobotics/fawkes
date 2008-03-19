
/***************************************************************************
 *  qa_bb_openall.h - BlackBoard interface QA
 *
 *  Created: Fri Jun 29 13:44:04 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/blackboard.h>
#include <blackboard/exceptions.h>
#include <blackboard/bbconfig.h>

#include <interfaces/test.h>

#include <core/exceptions/system.h>
#include <utils/logging/liblogger.h>

#include <signal.h>
#include <cstdlib>

#include <iostream>
#include <vector>

using namespace std;


int
main(int argc, char **argv)
{
  LibLogger::init();
  BlackBoard *bb = new BlackBoard();

  TestInterface *ti_writer_1;
  TestInterface *ti_writer_2;
  TestInterface *ti_writer_3;
  TestInterface *ti_writer_4;
  TestInterface *ti_writer_5;
  TestInterface *ti_writer_6;

  try {
    cout << "Opening interfaces.. " << flush;
    ti_writer_1 = bb->open_for_writing<TestInterface>("SomeID 1");
    ti_writer_2 = bb->open_for_writing<TestInterface>("SomeID 2");
    ti_writer_3 = bb->open_for_writing<TestInterface>("SomeID 3");
    ti_writer_4 = bb->open_for_writing<TestInterface>("AnotherID 1");
    ti_writer_5 = bb->open_for_writing<TestInterface>("AnotherID 2");
    ti_writer_6 = bb->open_for_writing<TestInterface>("AnotherID 3");
    cout << "success" << endl;
  } catch (Exception &e) {
    cout << "failed! Aborting" << endl;
    e.print_trace();
    exit(1);
  }

  std::list<Interface *> *readers = bb->open_all_of_type_for_reading("TestInterface");
  for (std::list<Interface *>::iterator i = readers->begin(); i != readers->end(); ++i) {
    printf("Opened reader for interface %s of type %s\n", (*i)->id(), (*i)->type());
    bb->close(*i);
  }
  delete readers;

  const char* prefix = "Another";
  readers = bb->open_all_of_type_for_reading("TestInterface", prefix);
  printf("Found %zu interfaces with prefix \"%s\"\n", readers->size(), prefix);
  for (std::list<Interface *>::iterator i = readers->begin(); i != readers->end(); ++i) {
    printf("Opened reader for interface %s of type %s\n", (*i)->id(), (*i)->type());
    bb->close(*i);
  }
  delete readers;
  
  bb->close(ti_writer_1);
  bb->close(ti_writer_2);
  bb->close(ti_writer_3);
  bb->close(ti_writer_4);
  bb->close(ti_writer_5);
  bb->close(ti_writer_6);

  delete bb;
  LibLogger::finalize();
}


/// @endcond
