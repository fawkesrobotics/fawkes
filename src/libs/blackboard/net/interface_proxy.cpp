
/***************************************************************************
 *  interface_proxy.cpp - BlackBoard interface proxy for RemoteBlackBoard
 *
 *  Created: Tue Mar 04 11:40:18 2008
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

#include <blackboard/net/interface_proxy.h>
#include <blackboard/internal/instance_factory.h>
#include <blackboard/net/messages.h>
#include <blackboard/internal/interface_mem_header.h>
#include <blackboard/internal/notifier.h>

#include <core/threading/refc_rwlock.h>
#include <logging/liblogger.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/message.h>

#include <cstdlib>
#include <cstring>
#include <arpa/inet.h>

namespace fawkes {

/** @class BlackBoardInterfaceProxy <blackboard/net/interface_proxy.h>
 * Interface proxy for remote BlackBoard.
 * This proxy is used internally by RemoteBlackBoard to interact with an interface
 * on the one side and the remote BlackBoard on the other side.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param client Fawkes network client
 * @param msg must be a MSG_BB_OPEN_SUCCESS message describing the interface in question
 * @param notifier BlackBoard notifier to use to notify of interface events
 * @param interface interface instance of the correct type, will be initialized in
 * this ctor and can be used afterwards.
 * @param writer true to make this a writing instance, false otherwise
 */
BlackBoardInterfaceProxy::BlackBoardInterfaceProxy(FawkesNetworkClient *client,
						   FawkesNetworkMessage *msg,
						   BlackBoardNotifier *notifier,
						   Interface *interface, bool writer)
{
  __fnc = client;
  if ( msg->msgid() != MSG_BB_OPEN_SUCCESS ) {
    throw Exception("Expected open success message");
  }

  void *payload = msg->payload();
  bb_iopensucc_msg_t *osm = (bb_iopensucc_msg_t *)payload;

  __notifier        = notifier;
  __interface       = interface;
  __instance_serial = ntohl(osm->serial);
  __has_writer      = osm->writer_readers & htonl(0x80000000);
  __num_readers     = ntohl(osm->writer_readers & htonl(0x7FFFFFFF));
  __data_size       = ntohl(osm->data_size);
  __clid            = msg->clid();
  __next_msg_id     = 1;

  if ( interface->datasize() != __data_size ) {
    // Boom, sizes do not match
    throw Exception("Network message does not carry chunk of expected size");
  }

  __rwlock     = new RefCountRWLock();
  __mem_chunk  = malloc(sizeof(interface_header_t) + __data_size);
  __data_chunk = (char *)__mem_chunk + sizeof(interface_header_t);
  memset(__mem_chunk, 0, sizeof(interface_header_t) + __data_size);
  memcpy(__data_chunk, (char *)payload + sizeof(bb_iopensucc_msg_t), __data_size);

  interface_header_t *ih = (interface_header_t *)__mem_chunk;

  strncpy(ih->type, interface->type(), __INTERFACE_TYPE_SIZE-1);
  strncpy(ih->id, interface->id(), __INTERFACE_ID_SIZE-1);
  memcpy(ih->hash, interface->hash(), __INTERFACE_HASH_SIZE);
  ih->flag_writer_active = (__has_writer ? 1 : 0);
  ih->num_readers = __num_readers;
  ih->refcount = 1;

  interface->set_instance_serial(__instance_serial);
  interface->set_memory(0, __mem_chunk, __data_chunk);
  interface->set_mediators(this, this);
  interface->set_readwrite(writer, __rwlock);
}

/** Destructor. */
BlackBoardInterfaceProxy::~BlackBoardInterfaceProxy()
{
  free(__mem_chunk);
}


/** Process MSG_BB_DATA_CHANGED message.
 * @param msg message to process.
 */
void
BlackBoardInterfaceProxy::process_data_changed(FawkesNetworkMessage *msg)
{
  if ( msg->msgid() != MSG_BB_DATA_CHANGED ) {
    LibLogger::log_error("BlackBoardInterfaceProxy", "Expected data changed BB message, but "
			 "received message of type %u, ignoring.", msg->msgid());
    return;
  }

  void *payload = msg->payload();
  bb_idata_msg_t *dm = (bb_idata_msg_t *)payload;
  if ( ntohl(dm->serial) != __instance_serial ) {
    LibLogger::log_error("BlackBoardInterfaceProxy", "Serial mismatch, expected %u, "
			 "but got %u, ignoring.", __instance_serial, ntohl(dm->serial));
    return;
  }

  if ( ntohl(dm->data_size) != __data_size ) {
    LibLogger::log_error("BlackBoardInterfaceProxy", "Data size mismatch, expected %zu, "
			 "but got %zu, ignoring.", __data_size, ntohl(dm->data_size));
    return;
  }

  memcpy(__data_chunk, (char *)payload + sizeof(bb_idata_msg_t), __data_size);

  __notifier->notify_of_data_change(__interface);
}


/** Process MSG_BB_INTERFACE message.
 * @param msg message to process.
 */
void
BlackBoardInterfaceProxy::process_interface_message(FawkesNetworkMessage *msg)
{
  if ( msg->msgid() != MSG_BB_INTERFACE_MESSAGE ) {
    LibLogger::log_error("BlackBoardInterfaceProxy", "Expected interface BB message, but "
			 "received message of type %u, ignoring.", msg->msgid());
    return;
  }

  void *payload = msg->payload();
  bb_imessage_msg_t *mm = (bb_imessage_msg_t *)payload;
  if ( ntohl(mm->serial) != __instance_serial ) {
    LibLogger::log_error("BlackBoardInterfaceProxy", "Serial mismatch (msg), expected %u, "
			 "but got %u, ignoring.", __instance_serial, ntohl(mm->serial));
    return;
  }

  if ( ! __interface->is_writer() ) {
    LibLogger::log_error("BlackBoardInterfaceProxy", "Received interface message, but this"
			 "is a reading instance (%s), ignoring.", __interface->uid());
    return;
  }

  try {
    Message *im = __interface->create_message(mm->msg_type);
    im->set_id(ntohl(mm->msgid));
    im->set_hops(ntohl(mm->hops) + 1);

    if (im->hops() > 1) {
      LibLogger::log_warn("BlackBoardInterfaceProxy", "Message IDs are not stable across more than one hop, "
			  "message of type %s for interface %s has %u hops",
			  im->type(), __interface->uid(), im->hops());
    }

    if ( ntohl(mm->data_size) != im->datasize() ) {
      LibLogger::log_error("BlackBoardInterfaceProxy", "Message data size mismatch, expected "
			   "%zu, but got %zu, ignoring.", im->datasize(), ntohl(mm->data_size));
      delete im;
      return;
    }

    im->set_from_chunk((char *)payload + sizeof(bb_imessage_msg_t));

    if ( __notifier->notify_of_message_received(__interface, im) ) {
      __interface->msgq_append(im);
      im->unref();
    }
  } catch (Exception &e) {
    e.append("Failed to enqueue interface message for %s, ignoring", __interface->uid());
    LibLogger::log_error("BlackBoardInterfaceProxy", e);
  }
}


/** Reader has been added.
 * @param event_serial instance serial of the interface that caused the event
 */
void
BlackBoardInterfaceProxy::reader_added(unsigned int event_serial)
{
  ++__num_readers;
  __notifier->notify_of_reader_added(__interface, event_serial);
}

/** Reader has been removed.
 * @param event_serial instance serial of the interface that caused the event
 */
void
BlackBoardInterfaceProxy::reader_removed(unsigned int event_serial)
{
  if ( __num_readers > 0 ) {
    --__num_readers;
  }
  __notifier->notify_of_reader_removed(__interface, event_serial);
}

/** Writer has been added.
 * @param event_serial instance serial of the interface that caused the event
 */
void
BlackBoardInterfaceProxy::writer_added(unsigned int event_serial)
{
  __has_writer = true;
  __notifier->notify_of_writer_added(__interface, event_serial);
}

/** Writer has been removed.
 * @param event_serial instance serial of the interface that caused the event
 */
void
BlackBoardInterfaceProxy::writer_removed(unsigned int event_serial)
{
  __has_writer = false;
  __notifier->notify_of_writer_removed(__interface, event_serial);
}


/** Get instance serial of interface.
 * @return instance serial
 */
unsigned int
BlackBoardInterfaceProxy::serial() const
{
  return __instance_serial;
}


/** Get client ID of assigned client.
 * @return client ID
 */
unsigned int
BlackBoardInterfaceProxy::clid() const
{
  return __instance_serial;
}

/** Get instance serial of interface.
 * @return instance serial
 */
Interface *
BlackBoardInterfaceProxy::interface() const
{
  return __interface;
}


/* InterfaceMediator */
bool
BlackBoardInterfaceProxy::exists_writer(const Interface *interface) const
{
  return __has_writer;
}

unsigned int
BlackBoardInterfaceProxy::num_readers(const Interface *interface) const
{
  return __num_readers;
}

std::list<std::string>
BlackBoardInterfaceProxy::readers(const Interface *interface) const
{
  throw NotImplementedException("Reader information not available for remote blackboard");
}


std::string
BlackBoardInterfaceProxy::writer(const Interface *interface) const
{
  throw NotImplementedException("Writer information not available for remote blackboard");
}

void
BlackBoardInterfaceProxy::notify_of_data_change(const Interface *interface)
{
  // need to send write message
  size_t payload_size = sizeof(bb_idata_msg_t) + interface->datasize();
  void *payload = malloc(payload_size);
  bb_idata_msg_t *dm = (bb_idata_msg_t *)payload;
  dm->serial = htonl(interface->serial());
  dm->data_size = htonl(interface->datasize());
  memcpy((char *)payload + sizeof(bb_idata_msg_t), interface->datachunk(),
	 interface->datasize());

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(__clid, FAWKES_CID_BLACKBOARD,
							MSG_BB_DATA_CHANGED,
							payload, payload_size);
  __fnc->enqueue(omsg);
}


/* MessageMediator */
void
BlackBoardInterfaceProxy::transmit(Message *message)
{
  // send out interface message
  size_t payload_size = sizeof(bb_imessage_msg_t) + message->datasize();
  void *payload = calloc(1, payload_size);
  bb_imessage_msg_t *dm = (bb_imessage_msg_t *)payload;
  dm->serial = htonl(__interface->serial());
  unsigned int msgid = next_msg_id();
  dm->msgid  = htonl(msgid);
  dm->hops   = htonl(message->hops());
  message->set_id(msgid);
  strncpy(dm->msg_type, message->type(), __INTERFACE_MESSAGE_TYPE_SIZE-1);
  dm->data_size = htonl(message->datasize());
  memcpy((char *)payload + sizeof(bb_imessage_msg_t), message->datachunk(),
	 message->datasize());

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(__clid, FAWKES_CID_BLACKBOARD,
							MSG_BB_INTERFACE_MESSAGE,
							payload, payload_size);
  __fnc->enqueue(omsg);
}

} // end namespace fawkes
