
/***************************************************************************
 *  fuse_transceiver.cpp - Fuse transceiver
 *
 *  Created: Wed Nov 14 13:30:34 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/net/fuse_transceiver.h>
#include <fvutils/net/fuse_message_queue.h>
#include <fvutils/net/fuse_message.h>
#include <netcomm/socket/stream.h>
#include <netcomm/utils/exceptions.h>

#include <netinet/in.h>
#include <cstdlib>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FuseNetworkTransceiver <fvutils/net/fuse_transceiver.h>
 * FUSE Network Transceiver.
 * Utility class that provides methods to send and receive messages via
 * the network. Operates on message queues and a given socket.
 *
 * @ingroup FUSE
 * @ingroup FireVision
 * @author Tim Niemueller
 */

/** Send messages.
 * @param s socket over which the data shall be transmitted.
 * @param msgq message queue that contains the messages that have to be sent
 * @exception ConnectionDiedException Thrown if any error occurs during the
 * operation since for any error the conncetion is considered dead.
 */
void
FuseNetworkTransceiver::send(StreamSocket *s, FuseNetworkMessageQueue *msgq)
{
  msgq->lock();
  try {
    while ( ! msgq->empty() ) {
      FuseNetworkMessage *m = msgq->front();
      m->pack();
      const FUSE_message_t &f = m->fmsg();
      unsigned int payload_size = m->payload_size();
      s->write(&(f.header), sizeof(f.header));
      s->write(f.payload, payload_size);
      m->unref();
      msgq->pop();
    }
  } catch (SocketException &e) {
    msgq->unlock();
    throw ConnectionDiedException("Write failed");
  }
  msgq->unlock();
}


/** Receive data.
 * This method receives all messages currently available from the network, or
 * a limited number depending on max_num_msgs. If max_num_msgs is 0 then all
 * messages are read. Note that on a busy connection this may cause recv() to
 * never return! The default is to return after 8 messages.
 * The messages are stored in the supplied message queue.
 * @param s socket to gather messages from
 * @param msgq message queue to store received messages in
 * @param max_num_msgs maximum number of messages to read from stream in one go.
 * @exception ConnectionDiedException Thrown if any error occurs during the
 * operation since for any error the conncetion is considered dead.
 */
void
FuseNetworkTransceiver::recv(StreamSocket *s, FuseNetworkMessageQueue *msgq,
			     unsigned int max_num_msgs)
{
  msgq->lock();

  try {
    unsigned int num_msgs = 0;
    do {
      FUSE_message_t msg;
      s->read(&(msg.header), sizeof(msg.header));

      unsigned int payload_size = ntohl(msg.header.payload_size);

      if ( payload_size > 0 ) {

	msg.payload = malloc(payload_size);
	s->read(msg.payload, payload_size);
      } else {
	msg.payload = NULL;
      }

      FuseNetworkMessage *m = new FuseNetworkMessage(&msg);
      msgq->push(m);

      ++num_msgs;
    } while ( s->available() && (num_msgs < max_num_msgs) );
  } catch (SocketException &e) {
    msgq->unlock();
    throw ConnectionDiedException("Read failed");
  }
  msgq->unlock();
}

} // end namespace firevision
