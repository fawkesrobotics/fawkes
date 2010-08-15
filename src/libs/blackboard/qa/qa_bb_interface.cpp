
/***************************************************************************
 *  qa_bb_interface.h - BlackBoard interface QA
 *
 *  Generated: Tue Oct 17 15:48:45 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/internal/memory_manager.h>
#include <blackboard/local.h>
#include <blackboard/exceptions.h>
#include <blackboard/bbconfig.h>

#include <interfaces/TestInterface.h>

#include <core/exceptions/system.h>

#include <signal.h>
#include <cstdlib>
#include <cstdio>

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

  LocalBlackBoard *lbb = new LocalBlackBoard(BLACKBOARD_MEMSIZE);

  BlackBoard *bb = lbb;
  const BlackBoardMemoryManager *mm = lbb->memory_manager();

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

  try {
    cout << "Trying to open second writer.. " << flush;
    TestInterface *ti_writer_two;
    ti_writer_two = bb->open_for_writing<TestInterface>("SomeID");
    cout << "BUG: Detection of second writer did NOT work!" << endl;
    exit(2);
  } catch (BlackBoardWriterActiveException &e) {
    cout << "exception caught as expected, detected and prevented second writer!" << endl;
  }

  cout << "Printing some meminfo ===============================================" << endl;
  cout << "Free chunks:" << endl;
  mm->print_free_chunks_info();
  cout << "Allocated chunks:" << endl;
  mm->print_allocated_chunks_info();
  mm->print_performance_info();
  cout << "End of meminfo ======================================================" << endl;

  try {
    cout << "Trying to open third writer.. " << flush;
    TestInterface *ti_writer_three;
    ti_writer_three = bb->open_for_writing<TestInterface>("AnotherID");
    cout << "No exception as expected, different ID ok!" << endl;
    bb->close(ti_writer_three);
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

  cout << "Reading value from reader interface.. " << flush;
  ti_reader->read();
  int val = ti_reader->test_int();
  if ( val == TestInterface::TEST_CONSTANT ) {
    cout << " success, value is " << ti_reader->test_int() << " as expected" << endl;
  } else {
    cout << " failure, value is " << ti_reader->test_int() << ", expected "
	 << TestInterface::TEST_CONSTANT << endl;
  }


  cout << "Iterating over reader interface.." << endl;
  InterfaceFieldIterator fi;
  for ( fi = ti_reader->fields(); fi != ti_reader->fields_end(); ++fi) {
    printf("Name: %20s  Type: %10s  Value: %s\n", fi.get_name(), fi.get_typename(), fi.get_value_string());
  }
  cout << "done" << endl;

  cout << "Harnessing interface by excessive reading and writing, use Ctrl-C to interrupt" << endl
       << "If you do not see any output everything is fine" << endl;
  while ( ! quit ) {
    int expval = ti_reader->test_int() + 1;
    //cout << "Writing value " << expval
    // << " into interface as TestInt" << endl;
    ti_writer->set_test_int( expval );
    try {
      ti_writer->write();
    } catch (InterfaceWriteDeniedException &e) {
      cout << "BUG: caught write denied exception" << endl;
      e.print_trace();
    }

    //cout << "Reading value from reader interface.. " << flush;
    ti_reader->read();
    int val = ti_reader->test_int();
    if ( val == expval ) {
      //cout << " success, value is " << ti_reader->test_int() << " as expected" << endl;
    } else {
      cout << " failure, value is " << ti_reader->test_int() << ", expected "
      	   << expval << endl;
    }

    usleep(10);
  }

  cout << "Tests done" << endl;

  bb->close(ti_reader);
  bb->close(ti_writer);

  delete bb;
}


/// @endcond
