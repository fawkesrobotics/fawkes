
/***************************************************************************
 *  qa_bb_listall.cpp - BlackBoard interface QA: list all
 *
 *  Created: Mon Mar 03 16:27:23 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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
#include <interface/interface_info.h>

#include <core/exceptions/system.h>

#include <signal.h>
#include <cstdlib>
#include <cstring>

#include <iostream>
#include <vector>

using namespace std;


bool quit = false;

void
signal_handler(int signum)
{
  quit = true;
}


#define NUM_CHUNKS 5

int
main(int argc, char **argv)
{

  signal(SIGINT, signal_handler);

  BlackBoard *bb = new BlackBoard(/* master */  true);
  
  TestInterface *ti_writer;
  TestInterface *ti_reader;

  try {
    cout << "Opening interfaces.. " << flush;
    ti_writer = bb->open_for_writing<TestInterface>("SomeID");
    ti_reader = bb->open_for_reading<TestInterface>("SomeID");
    cout << "success, " <<
            "writer hash=" << ti_writer->hash_printable() <<
            "  reader hash=" << ti_reader->hash_printable() << endl;
  } catch (Exception &e) {
    cout << "failed! Aborting" << endl;
    e.print_trace();
    exit(1);
  }

  cout << "Listing interfaces.." << endl;
  InterfaceInfoList *infl = bb->list_all();
  for (InterfaceInfoList::iterator i = infl->begin(); i != infl->end(); ++i) {
    const unsigned char *hash = (*i).hash();
    char phash[__INTERFACE_HASH_SIZE * 2 + 1];
    memset(phash, 0, sizeof(phash));
    for (unsigned int j = 0; j < __INTERFACE_HASH_SIZE; ++j) {
      sprintf(&phash[j * 2], "%02x", hash[j]);
    }
    printf("%s::%s (%s), w:%i  r:%u  s:%u\n",
	   (*i).type(), (*i).id(), phash, (*i).has_writer(),
	   (*i).num_readers(), (*i).serial());
  }
  delete infl;

  bb->close(ti_reader);
  bb->close(ti_writer);

  delete bb;
}


/// @endcond
