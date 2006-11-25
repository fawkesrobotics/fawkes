
/***************************************************************************
 *  transceiver.cpp - Fawkes transceiver
 *
 *  Created: Tue Nov 21 23:46:13 2006
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <netcomm/fawkes/transceiver.h>
#include <netcomm/fawkes/message.h>
#include <netcomm/fawkes/message_queue.h>
#include <netcomm/socket/stream.h>

#include <netinet/in.h>

/** @class ConnectionDiedException netcomm/fawkes/transceiver.h
 * Thrown if the connection died during an operation.
 */

/** Constructor.
 * @param msg message
 */
ConnectionDiedException::ConnectionDiedException(const char *msg)
  : Exception(msg)
{
}


/** @class FawkesNetworkTransceiver netcomm/fawkes/transceiver.h
 * Fawkes Network Transceiver.
 * Utility class that provides methods to send and receive messages via
 * the network. Operates on message queues and a given socket.
 *
 * @author Tim Niemueller
 */

/** Send messages.
 * @param s socket over which the data shall be transmitted.
 * @param msgq message queue that contains the messages that have to be sent
 * @param buffer the buffer will be used as outbound buffer. The buffer is
 * filled with as many message that fit into the buffer (determined by buffer_size
 * and the message sizes) and then it is sent over the network. This is done to
 * reduce the number of packets sent to reduce overhead.
 * @param buffer_size size of buffer
 * @exception ConnectionDiedException Thrown if any error occurs during the
 * operation since for any error the conncetion is considered dead.
 */
void
FawkesNetworkTransceiver::send(StreamSocket *s, FawkesNetworkMessageQueue *msgq,
			       void *buffer, unsigned int buffer_size)
{
  msgq->lock();
  try {
    unsigned int cur_size = 0;
    void *b = (char *)buffer + sizeof(fawkes_transfer_header_t);
    fawkes_transfer_header_t theader;
    while ( ! msgq->empty() ) {
      // we put messages into the buffer until the buffer size is reached, then we
      // write it
      FawkesNetworkMessage *m = msgq->front();
      const fawkes_message_t &f = m->msg();
      unsigned int payload_size = m->payload_size();
      if ( (sizeof(fawkes_transfer_header_t) + cur_size + sizeof(fawkes_message_t) + payload_size) > buffer_size ) {
	// flush buffer
	theader.size = htonl(cur_size);
	memcpy(buffer, &theader, sizeof(theader));
	s->write(buffer, cur_size + sizeof(fawkes_transfer_header_t));
	b = (char *)buffer + sizeof(fawkes_transfer_header_t);
	cur_size = 0;
      }
      memcpy(b, &(f.header), sizeof(f.header));
      b = (char *)b + sizeof(f.header);
      memcpy(b, f.payload, payload_size);
      b = (char *)b + payload_size;
      m->unref();
      cur_size += sizeof(f.header) + payload_size;
      msgq->pop();
    }
    if ( cur_size > 0 ) {
      // data needs to be written
      theader.size = htonl(cur_size);
      memcpy(buffer, &theader, sizeof(theader));
      s->write(buffer, cur_size + sizeof(fawkes_transfer_header_t));
    }
  } catch (SocketException &e) {
    msgq->unlock();
    throw ConnectionDiedException("Read failed");
  }
  msgq->unlock();
}


/** Receive data.
 * This method receives all messages currently available from the network.
 * The messages are stored in the supplied message queue.
 * @param s socket to gather messages from
 * @param msgq message queue to store received messages in
 * @exception ConnectionDiedException Thrown if any error occurs during the
 * operation since for any error the conncetion is considered dead.
 */
void
FawkesNetworkTransceiver::recv(StreamSocket *s, FawkesNetworkMessageQueue *msgq)
{
  unsigned int packet_size = 0;
  fawkes_transfer_header_t theader;
  msgq->lock();

  try {
    unsigned int read_bytes = 0; // without initial size
    s->read(&theader, sizeof(theader));
    // Now we know how many bytes to read and we can detect
    // errors, start reading messages
    packet_size = ntohl(theader.size);

    while (read_bytes < packet_size ) {

      fawkes_message_t msg;
      s->read(&(msg.header), sizeof(msg.header));
      read_bytes += sizeof(msg.header);

      unsigned int payload_size = ntohl(msg.header.payload_size);

      if ( payload_size > (packet_size - read_bytes)) {
	// this is obviously a problem
	throw Exception("Protocol error");
      }

      msg.payload = malloc(payload_size);
      s->read(msg.payload, payload_size);
      read_bytes += payload_size;

      FawkesNetworkMessage *m = new FawkesNetworkMessage(msg);
      msgq->push(m);
    }
  } catch (SocketException &e) {
    msgq->unlock();
    throw ConnectionDiedException("Read failed");
  }
  msgq->unlock();
}
