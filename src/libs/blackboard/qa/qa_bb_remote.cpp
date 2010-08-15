
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

#include <blackboard/local.h>
#include <blackboard/remote.h>
#include <blackboard/exceptions.h>
#include <blackboard/bbconfig.h>
#include <blackboard/interface_listener.h>

#include <interfaces/TestInterface.h>

#include <interface/interface_info.h>
#include <core/exceptions/system.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/server_thread.h>
#include <utils/time/time.h>

#include <signal.h>
#include <cstdlib>
#include <cstring>
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

void
test_messaging(TestInterface *ti_reader, TestInterface *ti_writer)
{
  while (! quit) {
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
	printf("Received message with ID %u (enqueue time: %s)\n", m2->id(), m2->time_enqueued()->str());
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
}

class SyncInterfaceListener : public fawkes::BlackBoardInterfaceListener
{
public:
  SyncInterfaceListener(fawkes::Interface *reader,
			fawkes::Interface *writer,
			fawkes::BlackBoard *reader_bb,
			fawkes::BlackBoard *writer_bb)
    : BlackBoardInterfaceListener("SyncInterfaceListener(%s-%s)", writer->uid(), reader->id())
  {
    __reader    = reader;
    __writer    = writer;
    __reader_bb = reader_bb;
    __writer_bb = writer_bb;

    bbil_add_data_interface(__reader);
    bbil_add_message_interface(__writer);

    __reader_bb->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
    __writer_bb->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);
  }


  /** Destructor. */
  ~SyncInterfaceListener()
  {
    __reader_bb->unregister_listener(this);
    __writer_bb->unregister_listener(this);
  }


  bool
  bb_interface_message_received(Interface *interface,
				Message *message) throw()
  {
    try {
      if ( interface == __writer ) {
	printf("%s: Forwarding message\n", bbil_name());
	Message *m = message->clone();
	m->set_hops(message->hops());
	m->ref();
	__reader->msgq_enqueue(m);
	message->set_id(m->id());
	m->unref();
	return false;
      } else {
	// Don't know why we were called, let 'em enqueue
	printf("%s: Message received for unknown interface\n", bbil_name());
	return true;
      }
    } catch (Exception &e) {
      printf("%s: Exception when message received\n", bbil_name());
      e.print_trace();
      return false;
    }
  }


  void
  bb_interface_data_changed(Interface *interface) throw()
  {
    try {
      if ( interface == __reader ) {
	//__logger->log_debug(bbil_name(), "Copying data");
	__reader->read();
	__writer->copy_values(__reader);
	__writer->write();
      } else {
	// Don't know why we were called, let 'em enqueue
	printf("%s: Data changed for unknown interface", bbil_name());
      }
    } catch (Exception &e) {
      printf("%s: Exception when data changed\n", bbil_name());
      e.print_trace();
    }
  }

 private:
  fawkes::Interface  *__writer;
  fawkes::Interface  *__reader;

  fawkes::BlackBoard *__writer_bb;
  fawkes::BlackBoard *__reader_bb;

};


int
main(int argc, char **argv)
{
  signal(SIGINT, signal_handler);

  LocalBlackBoard *llbb = new LocalBlackBoard(BLACKBOARD_MEMSIZE);
  BlackBoard *lbb = llbb;

  FawkesNetworkServerThread  *fns = new FawkesNetworkServerThread(1910);
  fns->start();

  llbb->start_nethandler(fns);

  BlackBoard *rbb = new RemoteBlackBoard("localhost", 1910);

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

  cout << endl << endl << "Starting MESSAGING tests" << endl 
       << "Press Ctrl-C to continue with next test" << endl << endl;

  ti_writer = lbb->open_for_writing<TestInterface>("Messaging");
  ti_reader = rbb->open_for_reading<TestInterface>("Messaging");

  printf("Writer serial: %u  shifted: %u\n", ti_writer->serial(), ti_writer->serial() << 16);
  printf("Reader serial: %u  shifted: %u\n", ti_reader->serial(), ti_reader->serial() << 16);

  test_messaging(ti_reader, ti_writer);

  rbb->close(ti_reader);
  lbb->close(ti_writer);

  cout << endl << endl << "Starting MESSAGING tests, doing repeater scenario" << endl 
       << "Press Ctrl-C to continue with next test" << endl << endl;
  quit = false;

  delete rbb;

  LocalBlackBoard *repllbb = new LocalBlackBoard(BLACKBOARD_MEMSIZE);

  FawkesNetworkServerThread  *repfns = new FawkesNetworkServerThread(1911);
  repfns->start();

  repllbb->start_nethandler(repfns);

  BlackBoard *rep_rbb = new RemoteBlackBoard("localhost", 1911);
  rbb = new RemoteBlackBoard("localhost", 1911);

  TestInterface *rep_reader;
  TestInterface *rep_writer;

  ti_writer = rbb->open_for_writing<TestInterface>("Messaging");
  ti_reader = lbb->open_for_reading<TestInterface>("Messaging");

  rep_reader = rep_rbb->open_for_reading<TestInterface>("Messaging");
  rep_writer = lbb->open_for_writing<TestInterface>("Messaging");

  printf("Writer serial: %u  shifted: %u\n", ti_writer->serial(), ti_writer->serial() << 16);
  printf("Reader serial: %u  shifted: %u\n", ti_reader->serial(), ti_reader->serial() << 16);

  SyncInterfaceListener *sil = new SyncInterfaceListener(rep_reader, rep_writer, rep_rbb, lbb);

  test_messaging(ti_reader, ti_writer);

  delete sil;
  lbb->close(ti_reader);
  rbb->close(ti_writer);
  rep_rbb->close(rep_reader);
  lbb->close(rep_writer);
  delete repllbb;
  delete rep_rbb;

  cout << "Tests done" << endl;

  delete rbb;
  delete llbb;
  delete fns;
}


/// @endcond
