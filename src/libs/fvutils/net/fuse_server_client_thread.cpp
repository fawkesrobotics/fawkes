
/***************************************************************************
 *  fuse_server_client_thread.cpp - client thread for FuseServer
 *
 *  Created: Tue Nov 13 20:00:55 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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
#include <logging/liblogger.h>

#include <netinet/in.h>
#include <cstring>
#include <cstdlib>
#include <unistd.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

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

  for (__lit = __luts.begin(); __lit != __luts.end(); ++__lit ) {
    delete __lit->second;
  }
  __luts.clear();

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


SharedMemoryImageBuffer *
FuseServerClientThread::get_shmimgbuf(const char *id)
{
  char tmp_image_id[IMAGE_ID_MAX_LENGTH + 1];
  tmp_image_id[IMAGE_ID_MAX_LENGTH] = 0;
  strncpy(tmp_image_id, id, IMAGE_ID_MAX_LENGTH);

  if ( (__bit = __buffers.find( tmp_image_id )) == __buffers.end() ) {
    // the buffer has not yet been opened
    try {
      SharedMemoryImageBuffer *b = new SharedMemoryImageBuffer(tmp_image_id);
      __buffers[tmp_image_id] = b;
      return b;
    } catch (Exception &e) {
      throw;
    }
  } else {
    return __bit->second;
  }
}


/** Process image request message.
 * @param m received message
 */
void
FuseServerClientThread::process_getimage_message(FuseNetworkMessage *m)
{
  FUSE_imagereq_message_t *irm = m->msg<FUSE_imagereq_message_t>();

  SharedMemoryImageBuffer *b;
  try {
    b = get_shmimgbuf(irm->image_id);
  } catch (Exception &e) {
    FuseNetworkMessage *nm = new FuseNetworkMessage(FUSE_MT_GET_IMAGE_FAILED,
						    m->payload(), m->payload_size(),
						    /* copy payload */ true);
    __outbound_queue->push(nm);
    return;
  }

  if ( irm->format == FUSE_IF_RAW ) {
    FuseImageContent *im = new FuseImageContent(b);
    __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_IMAGE, im));
  } else if ( irm->format == FUSE_IF_JPEG ) {
    if ( ! __jpeg_compressor) {
      __jpeg_compressor = new JpegImageCompressor();
      __jpeg_compressor->set_compression_destination(ImageCompressor::COMP_DEST_MEM);
    }
    b->lock_for_read();
    __jpeg_compressor->set_image_dimensions(b->width(), b->height());
    __jpeg_compressor->set_image_buffer(b->colorspace(), b->buffer());
    unsigned char *compressed_buffer = (unsigned char *)malloc(__jpeg_compressor->recommended_compressed_buffer_size());
    __jpeg_compressor->set_destination_buffer(compressed_buffer, __jpeg_compressor->recommended_compressed_buffer_size());
    __jpeg_compressor->compress();
    b->unlock();
    size_t compressed_buffer_size = __jpeg_compressor->compressed_size();
    long int sec = 0, usec = 0;
    b->capture_time(&sec, &usec);
    FuseImageContent *im = new FuseImageContent(FUSE_IF_JPEG, b->image_id(),
						compressed_buffer, compressed_buffer_size,
						CS_UNKNOWN, b->width(), b->height(),
						sec, usec);
    __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_IMAGE, im));
    free(compressed_buffer);
  } else {
    FuseNetworkMessage *nm = new FuseNetworkMessage(FUSE_MT_GET_IMAGE_FAILED,
						    m->payload(), m->payload_size(),
						    /* copy payload */ true);
    __outbound_queue->push(nm);
  }
}

/** Process image info request message.
 * @param m received message
 */
void
FuseServerClientThread::process_getimageinfo_message(FuseNetworkMessage *m)
{
  FUSE_imagedesc_message_t *idm = m->msg<FUSE_imagedesc_message_t>();

  SharedMemoryImageBuffer *b;
  try {
    b = get_shmimgbuf(idm->image_id);

    FUSE_imageinfo_t *ii = (FUSE_imageinfo_t *)calloc(1, sizeof(FUSE_imageinfo_t));
    
    strncpy(ii->image_id, b->image_id(), IMAGE_ID_MAX_LENGTH-1);
    ii->colorspace = htons(b->colorspace());
    ii->width = htonl(b->width());
    ii->height = htonl(b->height());
    ii->buffer_size = colorspace_buffer_size(b->colorspace(), b->width(), b->height());

    FuseNetworkMessage *nm = new FuseNetworkMessage(FUSE_MT_IMAGE_INFO,
						    ii, sizeof(FUSE_imageinfo_t));
    __outbound_queue->push(nm);
  } catch (Exception &e) {
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
    FuseLutContent *lm = new FuseLutContent(__lit->second);
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


/** Process LUT setting.
 * @param m received message
 */
void
FuseServerClientThread::process_setlut_message(FuseNetworkMessage *m)
{
  FuseLutContent *lc = m->msgc<FuseLutContent>();  
  FUSE_lutdesc_message_t *reply = (FUSE_lutdesc_message_t *)malloc(sizeof(FUSE_lutdesc_message_t));
  strncpy(reply->lut_id, lc->lut_id(), LUT_ID_MAX_LENGTH-1);
  // Currently we expect colormaps, so make sure we get sensible dimensions

  SharedMemoryLookupTable *b;
  if ( (__lit = __luts.find( lc->lut_id() )) != __luts.end() ) {
    // the buffer had already been opened
    b = __lit->second;
  } else {
    try {
      b = new SharedMemoryLookupTable(lc->lut_id(), /* read only */ false);
      __luts[lc->lut_id()] = b;
    } catch (Exception &e) {
      __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_SET_LUT_FAILED,
						    reply, sizeof(FUSE_lutdesc_message_t)));
      e.append("Cannot open shared memory lookup table %s", lc->lut_id());
      LibLogger::log_warn("FuseServerClientThread", e);
      delete lc;
      return;
    }
  }

  if ( (b->width() != lc->width())   ||
       (b->height() != lc->height()) ||
       (b->depth() != lc->depth())   ||
       (b->bytes_per_cell() != lc->bytes_per_cell()) ) {
    __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_SET_LUT_FAILED,
						  reply, sizeof(FUSE_lutdesc_message_t)));
    LibLogger::log_warn("FuseServerClientThread", "LUT upload: dimensions do not match. "
			"Existing (%u,%u,%u,%u) != uploaded (%u,%u,%u,%u)",
			b->width(), b->height(), b->depth(), b->bytes_per_cell(),
			lc->width(), lc->height(), lc->depth(), lc->bytes_per_cell());
  } else {
    b->set(lc->buffer());
    __outbound_queue->push(new FuseNetworkMessage(FUSE_MT_SET_LUT_SUCCEEDED,
						  reply, sizeof(FUSE_lutdesc_message_t)));
  }

  delete lc;
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
      llm->add_lutinfo(lh->lut_id(), lh->width(), lh->height(), lh->depth(), lh->bytes_per_cell());
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
      case FUSE_MT_GET_IMAGE_INFO:
	process_getimageinfo_message(m);
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
      case FUSE_MT_SET_LUT:
	process_setlut_message(m);
	break;
      default:
	throw Exception("Unknown message type received\n");
      }
    } catch (Exception &e) {
      e.append("FUSE protocol error");
      LibLogger::log_warn("FuseServerClientThread", e);
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
    try {
      // Data can be read
      recv();
      process_inbound();
    }
    catch (...) {
      __fuse_server->connection_died(this);
      __alive = false;
    }
  }

  if ( __alive ) {
    send();
  }
}

} // end namespace firevision
