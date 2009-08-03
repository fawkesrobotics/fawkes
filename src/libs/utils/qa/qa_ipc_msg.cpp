
/***************************************************************************
 *  qa_ipc_msg.h - QA for IPC message queues
 *
 *  Generated: Mon Sep 18 23:13:10 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

// Do not include in api reference
///@cond QA

#include <utils/ipc/msg.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#define QA_MTYPE 1

using namespace fawkes;

typedef struct {
  long mtype;
  char msg[20];
} simple_msg_t;

int
main( int argc, char **argv )
{
  // Create message queue, destroy on delete
  IPCMessageQueue *m1 = new IPCMessageQueue(".", 'A', true, true);

  // Open, do not create, do not destroy
  IPCMessageQueue *m2 = new IPCMessageQueue(".", 'A', false, false);

  for (unsigned int i = 0; i < 10; ++i) {
    simple_msg_t smsg;
    simple_msg_t rmsg;
    memset(&smsg, 0, sizeof(smsg));
    memset(&rmsg, 0, sizeof(rmsg));

    smsg.mtype = QA_MTYPE;
    sprintf(smsg.msg, "%u", i);

    m1->send((IPCMessageQueue::MessageStruct *)&smsg, sizeof(smsg));
    m2->recv(QA_MTYPE, (IPCMessageQueue::MessageStruct *)&rmsg, sizeof(rmsg));

    printf("Sent: %s     Received: %s\n", smsg.msg, rmsg.msg);
  }

  delete m2;
  delete m1;

  return 0;
}



/// @endcond
