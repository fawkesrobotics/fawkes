
/***************************************************************************
 *  qa_ipc_msg.h - QA for IPC message queues
 *
 *  Generated: Mon Sep 18 23:13:10 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

// Do not include in api reference
///@cond QA

#include <utils/ipc/msg.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#define MTYPE 1

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

    smsg.mtype = MTYPE;
    sprintf(smsg.msg, "%u", i);

    m1->send((IPCMessageQueue::MessageStruct *)&smsg, sizeof(smsg));
    m2->recv(MTYPE, (IPCMessageQueue::MessageStruct *)&rmsg, sizeof(rmsg));

    printf("Sent: %s     Received: %s\n", smsg.msg, rmsg.msg);
  }

  delete m2;
  delete m1;

  return 0;
}



/// @endcond
