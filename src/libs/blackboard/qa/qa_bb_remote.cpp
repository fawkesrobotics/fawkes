
/***************************************************************************
 *  qa_bb_remote.cpp - BlackBoard remote access QA
 *
 *  Created: Mon Mar 03 17:31:18 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <interfaces/TestInterface.h>

#include <interface/interface_info.h>
#include <core/exceptions/system.h>
#include <netcomm/fawkes/client.h>

#include <signal.h>
#include <cstdlib>
#include <cstring>

#include <iostream>
#include <vector>

using namespace std;
using namespace fawkes;


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

  FawkesNetworkClient *fnc = new FawkesNetworkClient("localhost", 1910);
  fnc->connect();

  BlackBoard *rbb = new RemoteBlackBoard(fnc);

  InterfaceInfoList *infl = rbb->list_all();
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

  //TestInterface *ti_writer;
  TestInterface *ti_reader;
  TestInterface *ti_writer;
  try {
    cout << "Opening interfaces.. " << flush;
    ti_writer = rbb->open_for_writing<TestInterface>("SomeID");
    ti_reader = rbb->open_for_reading<TestInterface>("SomeID");
    cout << "success, "
	 << "writer hash=" << ti_writer->hash_printable()
	 << "  reader hash=" << ti_reader->hash_printable()
	 << endl;
  } catch (Exception &e) {
    cout << "failed! Aborting" << endl;
    e.print_trace();
    exit(1);
  }

  try {
    cout << "Trying to open second writer.. " << flush;
    TestInterface *ti_writer_two;
    ti_writer_two = rbb->open_for_writing<TestInterface>("SomeID");
    cout << "BUG: Detection of second writer did NOT work!" << endl;
    exit(2);
  } catch (BlackBoardWriterActiveException &e) {
    cout << "exception caught as expected, detected and prevented second writer!" << endl;
  }

  try {
    cout << "Trying to open third writer.. " << flush;
    TestInterface *ti_writer_three;
    ti_writer_three = rbb->open_for_writing<TestInterface>("AnotherID");
    cout << "No exception as expected, different ID ok!" << endl;
    rbb->close(ti_writer_three);
  } catch (BlackBoardWriterActiveException &e) {
    cout << "BUG: Third writer with different ID detected as another writer!" << endl;
    exit(3);
  }

  cout << endl << endl
       << "Running data tests ==================================================" << endl;

  cout << "Writing initial value ("
       << TestInterface::TEST_CONSTANT << ") into interface as TestInt" << endl;
  ti_writer->set_test_int( TestInterface::TEST_CONSTANT );
  try {
    ti_writer->write();
  } catch (InterfaceWriteDeniedException &e) {
    cout << "BUG: caught write denied exception" << endl;
    e.print_trace();
  }

  cout << "Giving some time to have value processed" << endl;
  usleep(100000);

  cout << "Reading value from reader interface.. " << flush;
  ti_reader->read();
  int val = ti_reader->test_int();
  if ( val == TestInterface::TEST_CONSTANT ) {
    cout << " success, value is " << ti_reader->test_int() << " as expected" << endl;
  } else {
    cout << " failure, value is " << ti_reader->test_int() << ", expected "
	 << TestInterface::TEST_CONSTANT << endl;
  }

  cout << "Closing interfaces.. " << flush;
  try {
    rbb->close(ti_reader);
    rbb->close(ti_writer);
    cout << "done" << endl;
  } catch (Exception &e) {
    cout << "failed" << endl;
    e.print_trace();
  }

  delete rbb;
  fnc->disconnect();
  delete fnc;
}


/// @endcond
