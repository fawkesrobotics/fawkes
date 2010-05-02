
/***************************************************************************
 *  qa_bb_messaging.h - BlackBoard messaging QA
 *
 *  Generated: Tue Oct 31 15:36:19 2006
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

#include <blackboard/local.h>
#include <blackboard/remote.h>
#include <blackboard/exceptions.h>
#include <blackboard/bbconfig.h>

#include <interfaces/TestInterface.h>

#include <core/threading/thread.h>
#include <core/exceptions/system.h>
#include <utils/time/time.h>

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
#define BLACKBOARD_MEMSIZE 2 * 1024 * 1024
#define BLACKBOARD_MAGIC_TOKEN "FawkesBlackBoard"

int
main(int argc, char **argv)
{

  Thread::init_main();

  signal(SIGINT, signal_handler);

  BlackBoard *bb = new LocalBlackBoard(BLACKBOARD_MEMSIZE);
  //BlackBoard *bb = new RemoteBlackBoard("localhost", 1910);

  TestInterface *ti_writer;
  TestInterface *ti_reader;

  try {
    cout << "Opening interfaces.. " << flush;
    ti_writer = bb->open_for_writing<TestInterface>("SomeID");
    ti_reader = bb->open_for_reading<TestInterface>("SomeID");
    cout << "success" << endl;
  } catch (Exception &e) {
    cout << "failed! Aborting" << endl;
    e.print_trace();
    exit(1);
  }

  cout << "Writing initial value ("
       << TestInterface::TEST_CONSTANT << ") into interface as TestInt" << endl;
  ti_writer->set_test_int( 5 );
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

  printf("Reader instance serial: %u\n", ti_reader->serial());

  cout << "Harnessing message queues by excessively sending messages" << endl
       << "Press Ctrl-C to stop testing. No output means everything is fine" << endl;
  while ( ! quit ) {
    int expval = ti_reader->test_int() + 1;
    TestInterface::SetTestIntMessage *m = new TestInterface::SetTestIntMessage(expval);
    unsigned int msgid = ti_reader->msgq_enqueue(m);
    printf("Sent with message ID %u\n", msgid);

    if ( ti_writer->msgq_size() > 1 ) {
      cout << "Error, more than one message! flushing." << endl;
      ti_writer->msgq_flush();
    }

    usleep(100000);

    if ( ti_writer->msgq_first() != NULL ) {
      if ( ti_writer->msgq_first_is<TestInterface::SetTestStringMessage>() ) {
	TestInterface::SetTestStringMessage *msg = ti_writer->msgq_first(msg);
	printf("Received message of ID %u, Message improperly detected to be a SetTestStringMessage\n", msg->id());
      }
      if ( ti_writer->msgq_first_is<TestInterface::SetTestIntMessage>() ) {
	TestInterface::SetTestIntMessage *m2 = ti_writer->msgq_first<TestInterface::SetTestIntMessage>();
	printf("Received message with ID %u (enqueue time: %s)\n", m2->id(),
	       m2->time_enqueued()->str());
	ti_writer->set_test_int( m2->test_int() );
	try {
	  ti_writer->write();
	} catch (InterfaceWriteDeniedException &e) {
	  cout << "BUG: caught write denied exception" << endl;
	  e.print_trace();
	}
	ti_writer->msgq_pop();
      } else {
	cout << "Illegal message '" << ti_writer->msgq_first()->type() << "' type received" << endl;
      }

      usleep(100000);

      //cout << "Reading value from reader interface.. " << flush;
      ti_reader->read();
      int val = ti_reader->test_int();
      if ( val == expval ) {
	//cout << " success, value is " << ti_reader->test_int() << " as expected" << endl;
      } else {
	cout << " failure, value is " << ti_reader->test_int() << ", expected "
	     << expval << endl;
      }
    } else {
      printf("No message in queue, if network test this means the message was dropped\n");
    }

    usleep(10);
  }

  bb->close(ti_reader);
  bb->close(ti_writer);

  delete bb;

  cout << "Tests done" << endl;

  Thread::destroy_main();
}


/// @endcond
