
/***************************************************************************
 *  fuse_server_client_thread.cpp - client thread for FuseServer
 *
 *  Created: Tue Nov 13 20:00:55 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/net/fuse_server_client_thread.h>

#include <fvutils/net/fuse_server.h>
#include <fvutils/net/fuse_server.h>
#include <fvutils/net/fuse_transceiver.h>
#include <fvutils/net/fuse_message_queue.h>
#include <fvutils/net/fuse_image_content.h>
#include <fvutils/net/fuse_lut_content.h>
#include <fvutils/net/fuse_imagelist_content.h>
#include <fvutils/net/fuse_lutlist_content.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/ipc/shm_lut.h>

#include <core/exceptions/system.h>
#include <netcomm/socket/stream.h>
#include <netcomm/utils/exceptions.h>

#include <netinet/in.h>

/** @class FuseServerClientThread <fvutils/net/fuse_server_client_thread.h>
 * FUSE Server Client Thread.
 * This thread is instantiated and started for each client that connects to a
 * FuseServer.
 * @ingroup FUSE
 * @ingroup FireVision
 * @author Tim Niemueller
 */

/** Constructor.
 * @param fuse_server parent FUSE server
 * @param s socket to client
 */
FuseServerClientThread::FuseServerClientThread(FuseServer *fuse_server, StreamSocket *s)
  : Thread("FuseServerClientThread")
{
  __fuse_server = fuse_server;
  __socket = s;

  __inbound_queue  = new FuseNetworkMessageQueue();
  __outbound_queue  = new FuseNetworkMessageQueue();

  FUSE_greeting_message_t *greetmsg = (FUSE_greeting_message_t *)malloc(sizeof(FUSE_greeting_message_t));
  greetmsg->version = htonl(FUSE_CURRENT_VERSION);
  __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_GREETING,
						greetmsg, sizeof(FUSE_greeting_message_t)));
}


/** Destructor. */
FuseServerClientThread::~FuseServerClientThread()
{
  delete __socket;

  while ( ! __inbound_queue->empty() ) {
    FuseNetworkMessage *m = __inbound_queue->front();
    m->unref();
    __inbound_queue->pop();
  }

  while ( ! __outbound_queue->empty() ) {
    FuseNetworkMessage *m = __outbound_queue->front();
    m->unref();
    __outbound_queue->pop();
  }

  delete __inbound_queue;
  delete __outbound_queue;
}


/** Send all messages in outbound queue. */
void
FuseServerClientThread::send()
{
  if ( ! __outbound_queue->empty() ) {
    try {
      FuseNetworkTransceiver::send(__socket, __outbound_queue);
    } catch (Exception &e) {
      __fuse_server->connection_died(this);
      exit();
    }
  }
}


/** Receive data.
 * Receives data from the network if there is any and then processes all
 * inbound messages.
 */
void
FuseServerClientThread::recv()
{
  try {
    FuseNetworkTransceiver::recv(__socket, __inbound_queue);
  } catch (ConnectionDiedException &e) {
    __socket->close();
    __fuse_server->connection_died(this);
    exit();
  }
}


/** Process greeting message.
 * @param m received message
 */
void
FuseServerClientThread::process_greeting_message(FuseNetworkMessage *m)
{
  FUSE_greeting_message_t *gm = m->msg<FUSE_greeting_message_t>();
  if ( ntohl(gm->version) != FUSE_CURRENT_VERSION ) {
    throw Exception("Invalid version on other side");
  }
}


/** Process image request message.
 * @param m received message
 */
void
FuseServerClientThread::process_getimage_message(FuseNetworkMessage *m)
{
  FUSE_imagedesc_message_t *idm = m->msg<FUSE_imagedesc_message_t>();

  char tmp_image_id[IMAGE_ID_MAX_LENGTH + 1];
  tmp_image_id[IMAGE_ID_MAX_LENGTH] = 0;
  strncpy(tmp_image_id, idm->image_id, IMAGE_ID_MAX_LENGTH);

  if ( (__bit = __buffers.find( tmp_image_id )) != __buffers.end() ) {
    // the buffer had already be opened
    FuseImageContent *im = new FuseImageContent((*__bit).second);
    __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_IMAGE, im));
  } else {
    try {
      SharedMemoryImageBuffer *b = new SharedMemoryImageBuffer(tmp_image_id);
      __buffers[tmp_image_id] = b;
      FuseImageContent *im = new FuseImageContent(b);
      __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_IMAGE, im));
    } catch (Exception &e) {
      // could not open the shared memory segment for some reason, send failure
      FuseNetworkMessage *nm = new FuseNetworkMessage(FUSE_MT_GET_IMAGE_FAILED,
						      m->payload(), m->payload_size(),
						      /* copy payload */ true);
      __outbound_queue->push(nm);
    }
  }
}


/** Process LUT request message.
 * @param m received message
 */
void
FuseServerClientThread::process_getlut_message(FuseNetworkMessage *m)
{
  FUSE_lutdesc_message_t *idm = m->msg<FUSE_lutdesc_message_t>();

  char tmp_lut_id[LUT_ID_MAX_LENGTH + 1];
  tmp_lut_id[LUT_ID_MAX_LENGTH] = 0;
  strncpy(tmp_lut_id, idm->lut_id, LUT_ID_MAX_LENGTH);

  if ( (__lit = __luts.find( tmp_lut_id )) != __luts.end() ) {
    // the buffer had already be opened
    FuseLutContent *lm = new FuseLutContent((*__lit).second);
    __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_LUT, lm));
  } else {
    try {
      SharedMemoryLookupTable *b = new SharedMemoryLookupTable(tmp_lut_id);
      __luts[tmp_lut_id] = b;
      FuseLutContent *lm = new FuseLutContent(b);
      __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_LUT, lm));
    } catch (Exception &e) {
      // could not open the shared memory segment for some reason, send failure
      FuseNetworkMessage *nm = new FuseNetworkMessage(FUSE_MT_GET_LUT_FAILED,
						      m->payload(), m->payload_size(),
						      /* copy payload */ true);
      __outbound_queue->push(nm);
    }
  }
}


/** Process image list request message.
 * @param m received message
 */
void
FuseServerClientThread::process_getimagelist_message(FuseNetworkMessage *m)
{
  FuseImageListContent *ilm = new FuseImageListContent();

  SharedMemoryImageBufferHeader *h      = new SharedMemoryImageBufferHeader();
  SharedMemory::SharedMemoryIterator i = SharedMemory::find(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, h);
  SharedMemory::SharedMemoryIterator endi = SharedMemory::end();
							    
  while ( i != endi ) {
    const SharedMemoryImageBufferHeader *ih = dynamic_cast<const SharedMemoryImageBufferHeader *>(*i);
    if ( ih ) {
      ilm->add_imageinfo(ih->image_id(), ih->colorspace(), ih->width(), ih->height());
    }

    ++i;
  }

  delete h;

  __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_IMAGE_LIST, ilm));
}


/** Process LUT list request message.
 * @param m received message
 */
void
FuseServerClientThread::process_getlutlist_message(FuseNetworkMessage *m)
{
  FuseLutListContent *llm = new FuseLutListContent();

  SharedMemoryLookupTableHeader *h = new SharedMemoryLookupTableHeader();
  SharedMemory::SharedMemoryIterator i = SharedMemory::find(FIREVISION_SHM_LUT_MAGIC_TOKEN, h);
  SharedMemory::SharedMemoryIterator endi = SharedMemory::end();
							    
  while ( i != endi ) {
    const SharedMemoryLookupTableHeader *lh = dynamic_cast<const SharedMemoryLookupTableHeader *>(*i);
    if ( lh ) {
      llm->add_lutinfo(lh->lut_id(), lh->width(), lh->height(), lh->bytes_per_cell());
    }

    ++i;
  }

  delete h;

  __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_LUT_LIST, llm));
}


/** Process inbound messages. */
void
FuseServerClientThread::process_inbound()
{
  __inbound_queue->lock();
  while ( ! __inbound_queue->empty() ) {
    FuseNetworkMessage *m = __inbound_queue->front();

    try {
      switch (m->type()) {
      case FUSE_MT_GREETING:
	process_greeting_message(m);
	break;
      case FUSE_MT_GET_IMAGE:
	process_getimage_message(m);
	break;
      case FUSE_MT_GET_IMAGE_LIST:
	process_getimagelist_message(m);
	break;
      case FUSE_MT_GET_LUT_LIST:
	process_getlutlist_message(m);
	break;
      case FUSE_MT_GET_LUT:
	process_getlut_message(m);
	break;
      default:
	throw Exception("Unknown message type received\n");
      }
    } catch (Exception &e) {
      __fuse_server->connection_died(this);
      exit();
    }

    m->unref();
    __inbound_queue->pop();
  }
  __inbound_queue->unlock();
}


void
FuseServerClientThread::loop()
{
  short p = 0;
  try {
    p = __socket->poll(10); // block for up to 10 ms
  } catch (InterruptedException &e) {
    // we just ignore this and try it again
    return;
  }

  if ( (p & Socket::POLL_ERR) ||
       (p & Socket::POLL_HUP) ||
       (p & Socket::POLL_RDHUP)) {
    __fuse_server->connection_died(this);
    exit();
  } else if ( p & Socket::POLL_IN ) {
    // Data can be read
    recv();
    process_inbound();
  }

  send();
}
