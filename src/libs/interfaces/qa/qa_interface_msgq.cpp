
/***************************************************************************
 *  qa_interface_msgq.cpp - QA for interface message queue
 *
 *  Created: Tue Oct 24 14:34:40 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

/// @cond QA

#include <interface/message_queue.h>
#include <iostream>

#include <interfaces/test.h>
#include <typeinfo>

using namespace std;
using namespace fawkes;

void
printMessages(MessageQueue *mq)
{
  cout << "Iterating through messages:" << endl;
  MessageQueue::MessageIterator i;
  mq->lock();
  for ( i = mq->begin(); i != mq->end(); ++i) {
    if ( i.is<TestInterface::SetTestIntMessage>() ) {
      cout << "Message " << i.id() << " int: "
	   << i.get<TestInterface::SetTestIntMessage>()->test_int()
	   << "   type: " << typeid((*i)).name()
	   << endl;
    } else if ( i.is<Message>() ) {
      cout << "It's just a message" << endl;
    } else {
      cout << "Message " << i.id() << " is not of correct type, it is " << typeid((*i)).name() << endl;
    }
  }
  mq->unlock();
}

int
main(int argc, char **argv)
{

  unsigned int id;
  MessageQueue *mq = new MessageQueue();

  cout << "Message queue initialized, now appending three test messages" << endl;

  TestInterface::SetTestIntMessage *m1 = new TestInterface::SetTestIntMessage(1);
  TestInterface::SetTestIntMessage *m2 = new TestInterface::SetTestIntMessage(2);
  TestInterface::SetTestIntMessage *m3 = new TestInterface::SetTestIntMessage(3);
  TestInterface::SetTestStringMessage *m4 = new TestInterface::SetTestStringMessage();
  id = mq->append(m1);
  cout << "m1 added with id " << id << endl;
  id = mq->append(m1);
  cout << "m1 added with id " << id << endl;
  id = mq->append(m2);
  cout << "m2 added with id " << id << endl;
  id = mq->append(m3);
  cout << "m3 added with id " << id << endl;
  id = mq->append(m4);
  cout << "m4 (of different type!) added with id " << id << endl;

  cout << "Size is now " << mq->size() << endl;

  cout << "Unreferencing messages" << endl;
  m1->unref();
  m2->unref();
  m3->unref();
  m4->unref();

  cout << "Appending m1 again" << endl;
  id = mq->append(m1);
  cout << "m1 added with id " << id << endl;
  cout << "Size is now " << mq->size() << endl;
  cout << "m1 refcount is now " << m1->refcount() << endl;

  printMessages(mq);

  cout << "Now removing message m1 once" << endl;
  mq->remove(m1);
  printMessages(mq);


  cout << "m1 has refcount " << m1->refcount() << endl;

  cout << "Now removing message m2" << endl;
  mq->remove(m2);
  printMessages(mq);

  cout << "Now removing message m4" << endl;
  mq->remove(m4);
  printMessages(mq);

  cout << "Size is now " << mq->size() << endl;

  printMessages(mq);

  delete mq;
  // messages will be erased when enqueued in mq!

}

/// @endcond
