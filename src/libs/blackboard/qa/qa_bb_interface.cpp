
/***************************************************************************
 *  qa_bb_interface.h - BlackBoard interface QA
 *
 *  Generated: Tue Oct 17 15:48:45 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/memory_manager.h>
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

  BlackBoardInterfaceManager *im = new BlackBoardInterfaceManager(/* master */  true);
  const BlackBoardMemoryManager *mm = im->memory_manager();

  TestInterface *ti_writer;
  TestInterface *ti_reader;

  try {
    cout << "Opening interfaces.. " << flush;
    ti_writer = im->open_for_writing<TestInterface>("SomeID");
    ti_reader = im->open_for_reading<TestInterface>("SomeID");
    cout << "success" << endl;
  } catch (Exception &e) {
    cout << "failed! Aborting" << endl;
    e.printTrace();
    exit(1);
  }

  try {
    cout << "Trying to open second writer.. " << flush;
    TestInterface *ti_writer_two;
    ti_writer_two = im->open_for_writing<TestInterface>("SomeID");
    cout << "BUG: Detection of second writer did NOT work!" << endl;
    exit(2);
  } catch (BlackBoardWriterActiveException &e) {
    cout << "exception caught as expected, detected and prevented second writer!" << endl;
  }

  cout << "Printing some meminfo ===============================================" << endl;
  cout << "Free chunks:" << endl;
  mm->printFreeChunksInfo();
  cout << "Allocated chunks:" << endl;
  mm->printAllocatedChunksInfo();
  mm->printPerformanceInfo();
  cout << "End of meminfo ======================================================" << endl;

  try {
    cout << "Trying to open third writer.. " << flush;
    TestInterface *ti_writer_three;
    ti_writer_three = im->open_for_writing<TestInterface>("AnotherID");
    cout << "No exception as expected, different ID ok!" << endl;
    im->close(ti_writer_three);
  } catch (BlackBoardWriterActiveException &e) {
    cout << "BUG: Third writer with different ID detected as another writer!" << endl;
    exit(3);
  }

  cout << endl << endl
       << "Running data tests ==================================================" << endl;

  cout << "Writing initial value ("
       << TestInterface::TEST_CONSTANT << ") into interface as TestInt" << endl;
  ti_writer->setTestInt( 5 );
  try {
    ti_writer->write();
  } catch (InterfaceWriteDeniedException &e) {
    cout << "BUG: caught write denied exception" << endl;
    e.printTrace();
  }

  cout << "Reading value from reader interface.. " << flush;
  ti_reader->read();
  int val = ti_reader->getTestInt();
  if ( val == TestInterface::TEST_CONSTANT ) {
    cout << " success, value is " << ti_reader->getTestInt() << " as expected" << endl;
  } else {
    cout << " failure, value is " << ti_reader->getTestInt() << ", expected "
	 << TestInterface::TEST_CONSTANT << endl;
  }

  while ( ! quit ) {
    int expval = ti_reader->getTestInt() + 1;
    //cout << "Writing value " << expval
    // << " into interface as TestInt" << endl;
    ti_writer->setTestInt( expval );
    try {
      ti_writer->write();
    } catch (InterfaceWriteDeniedException &e) {
      cout << "BUG: caught write denied exception" << endl;
      e.printTrace();
    }

    //cout << "Reading value from reader interface.. " << flush;
    ti_reader->read();
    int val = ti_reader->getTestInt();
    if ( val == expval ) {
      //cout << " success, value is " << ti_reader->getTestInt() << " as expected" << endl;
    } else {
      cout << " failure, value is " << ti_reader->getTestInt() << ", expected "
      	   << expval << endl;
    }

    usleep(10);
  }

  im->close(ti_reader);
  im->close(ti_writer);

  delete im;
}


/// @endcond
