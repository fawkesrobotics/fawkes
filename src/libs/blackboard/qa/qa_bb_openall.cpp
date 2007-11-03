
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

#include <blackboard/interface_manager.h>
#include <blackboard/exceptions.h>
#include <blackboard/bbconfig.h>

#include <interfaces/test.h>

#include <core/exceptions/system.h>

#include <signal.h>
#include <unistd.h>

#include <iostream>
#include <vector>

using namespace std;


int
main(int argc, char **argv)
{
  BlackBoardInterfaceManager *im = new BlackBoardInterfaceManager(/* master */  true);

  TestInterface *ti_writer_1;
  TestInterface *ti_writer_2;
  TestInterface *ti_writer_3;

  try {
    cout << "Opening interfaces.. " << flush;
    ti_writer_1 = im->open_for_writing<TestInterface>("SomeID 1");
    ti_writer_2 = im->open_for_writing<TestInterface>("SomeID 2");
    ti_writer_3 = im->open_for_writing<TestInterface>("SomeID 3");
    cout << "success" << endl;
  } catch (Exception &e) {
    cout << "failed! Aborting" << endl;
    e.print_trace();
    exit(1);
  }

  std::list<Interface *> *readers = im->open_all_of_type_for_reading("TestInterface");
  for (std::list<Interface *>::iterator i = readers->begin(); i != readers->end(); ++i) {
    printf("Opened reader for interface %s of type %s\n", (*i)->id(), (*i)->type());
    im->close(*i);
  }
  delete readers;

  im->close(ti_writer_1);
  im->close(ti_writer_2);
  im->close(ti_writer_3);

  delete im;
}


/// @endcond
