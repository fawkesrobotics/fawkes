
/***************************************************************************
 *  qa_bb_buffers.h - BlackBoard interface QA
 *
 *  Generated: Tue May 24 23:39:22 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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


int
main(int argc, char **argv)
{

  signal(SIGINT, signal_handler);

  LocalBlackBoard *lbb = new LocalBlackBoard(BLACKBOARD_MEMSIZE);

  BlackBoard *bb = lbb;

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

  cout << endl << endl
       << "Running data tests =================================================="
       << endl;

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

  cout << "Resizing buffer.. " << flush;
  try {
    ti_reader->resize_buffers(1);
    ti_reader->copy_private_to_buffer(0);
  } catch (Exception &e) {
    cout << "ERROR: Resizing failed, exception follows" << endl;
    e.print_trace();
    throw;
  }


  cout << "Testing buffers, use Ctrl-C to interrupt" << endl
       << "If you do not see any output everything is fine" << endl;
  while ( ! quit ) {

    //cout << "Writing value " << expval
    // << " into interface as TestInt" << endl;
    ti_writer->set_test_int( ti_writer->test_int() + 1 );
    try {
      ti_writer->write();
    } catch (InterfaceWriteDeniedException &e) {
      cout << "BUG: caught write denied exception" << endl;
      e.print_trace();
    }

    //cout << "Reading value from reader interface.. " << flush;
    ti_reader->read();
    int rval = ti_reader->test_int();
    int wval = ti_writer->test_int();

    ti_reader->read_from_buffer(0);
    int bval = ti_reader->test_int();

    if ( rval != wval ) {
      cout << " failure, reader value is " << rval << ", writer has "
	   << wval << endl;
    }

    if ( rval != bval + 1 ) {
      cout << " failure, reader value is " << rval << ", buffer has "
	   << bval << endl;
    }

    // could to copy_shared as well, but that is a little less predictable in
    // the case of concurrent writers, hence we want people to copy and paste
    // this version.
    ti_reader->read();
    ti_reader->copy_private_to_buffer(0);

    usleep(10);
  }

  cout << "Tests done" << endl;

  bb->close(ti_reader);
  bb->close(ti_writer);

  delete bb;
}


/// @endcond
