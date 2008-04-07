
/***************************************************************************
 *  fuse_server_client_thread.cpp - client thread for FuseServer
 *
 *  Created: Tue Nov 13 20:00:55 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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
#include <fvutils/compression/jpeg_compressor.h>

#include <core/exceptions/system.h>
#include <netcomm/socket/stream.h>
#include <netcomm/utils/exceptions.h>

#include <netinet/in.h>
#include <cstring>
#include <cstdlib>

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
  __jpeg_compressor = NULL;

  __inbound_queue  = new FuseNetworkMessageQueue();
  __outbound_queue  = new FuseNetworkMessageQueue();

  FUSE_greeting_message_t *greetmsg = (FUSE_greeting_message_t *)malloc(sizeof(FUSE_greeting_message_t));
  greetmsg->version = htonl(FUSE_CURRENT_VERSION);
  __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_GREETING,
						greetmsg, sizeof(FUSE_greeting_message_t)));

  __alive = true;
}


/** Destructor. */
FuseServerClientThread::~FuseServerClientThread()
{
  delete __socket;
  delete __jpeg_compressor;

  for (__bit = __buffers.begin(); __bit != __buffers.end(); ++__bit) {
    delete __bit->second;
  }
  __buffers.clear();

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
      __alive = false;
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
    __alive = false;
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
  FUSE_imagereq_message_t *irm = m->msg<FUSE_imagereq_message_t>();

  char tmp_image_id[IMAGE_ID_MAX_LENGTH + 1];
  tmp_image_id[IMAGE_ID_MAX_LENGTH] = 0;
  strncpy(tmp_image_id, irm->image_id, IMAGE_ID_MAX_LENGTH);

  if ( (__bit = __buffers.find( tmp_image_id )) == __buffers.end() ) {
    // the buffer has not yet been opened
    try {
      SharedMemoryImageBuffer *b = new SharedMemoryImageBuffer(tmp_image_id);
      __buffers[tmp_image_id] = b;
    } catch (Exception &e) {
      // ignored
    }
  }

  if ( (__bit = __buffers.find( tmp_image_id )) != __buffers.end() ) {
    // the buffer had already be opened
    SharedMemoryImageBuffer *eb = (*__bit).second;
    if ( irm->format == FUSE_IF_RAW ) {
      FuseImageContent *im = new FuseImageContent(eb);
      __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_IMAGE, im));
    } else if ( irm->format == FUSE_IF_JPEG ) {
      if ( ! __jpeg_compressor) {
	__jpeg_compressor = new JpegImageCompressor();
	__jpeg_compressor->set_compression_destination(ImageCompressor::COMP_DEST_MEM);
      }
      eb->lock_for_read();
      __jpeg_compressor->set_image_dimensions(eb->width(), eb->height());
      __jpeg_compressor->set_image_buffer(eb->colorspace(), eb->buffer());
      unsigned char *compressed_buffer = (unsigned char *)malloc(__jpeg_compressor->recommended_compressed_buffer_size());
      __jpeg_compressor->set_destination_buffer(compressed_buffer, __jpeg_compressor->recommended_compressed_buffer_size());
      __jpeg_compressor->compress();
      eb->unlock();
      size_t compressed_buffer_size = __jpeg_compressor->compressed_size();
      FuseImageContent *im = new FuseImageContent(FUSE_IF_JPEG, eb->image_id(),
						  compressed_buffer, compressed_buffer_size,
						  CS_UNKNOWN, eb->width(), eb->height());
      __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_IMAGE, im));
      free(compressed_buffer);
    } else {
      FuseNetworkMessage *nm = new FuseNetworkMessage(FUSE_MT_GET_IMAGE_FAILED,
						      m->payload(), m->payload_size(),
						      /* copy payload */ true);
      __outbound_queue->push(nm);
    }
  } else {
    // could not open the shared memory segment for some reason, send failure
    FuseNetworkMessage *nm = new FuseNetworkMessage(FUSE_MT_GET_IMAGE_FAILED,
						    m->payload(), m->payload_size(),
						    /* copy payload */ true);
    __outbound_queue->push(nm);
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
      __alive = false;
    }

    m->unref();
    __inbound_queue->pop();
  }
  __inbound_queue->unlock();
}


void
FuseServerClientThread::loop()
{
  if ( ! __alive ) {
    usleep(10000);
    return;
  }

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
    __alive = false;
  } else if ( p & Socket::POLL_IN ) {
    // Data can be read
    recv();
    process_inbound();
  }

  if ( __alive ) {
    send();
  }
}
