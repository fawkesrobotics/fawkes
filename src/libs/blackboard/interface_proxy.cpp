
/***************************************************************************
 *  interface_proxy.cpp - BlackBoard interface proxy for RemoteBlackBoard
 *
 *  Created: Tue Mar 04 11:40:18 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/interface_proxy.h>
#include <blackboard/instance_factory.h>
#include <blackboard/net_messages.h>
#include <blackboard/interface_mem_header.h>
#include <blackboard/notifier.h>

#include <core/threading/refc_rwlock.h>
#include <utils/logging/liblogger.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/message.h>

#include <cstdlib>
#include <cstring>

/** @class BlackBoardInterfaceProxy <blackboard/interface_proxy.h>
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

  __notifier = notifier;
  __interface = interface;
  __instance_serial = osm->serial;
  __has_writer = (osm->has_writer == 1);
  __num_readers = osm->num_readers;
  __data_size = osm->data_size;
  __clid = msg->clid();

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

  strncpy(ih->type, interface->type(), __INTERFACE_TYPE_SIZE);
  strncpy(ih->id, interface->id(), __INTERFACE_ID_SIZE);
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
  if ( dm->serial != __instance_serial ) {
    LibLogger::log_error("BlackBoardInterfaceProxy", "Serial mismatch, expected %u, "
			 "but got %u, ignoring.", __instance_serial, dm->serial);
    return;
  }

  if ( dm->data_size != __data_size ) {
    LibLogger::log_error("BlackBoardInterfaceProxy", "Data size mismatch, expected %zu, "
			 "but got %zu, ignoring.", __data_size, dm->data_size);
    return;
  }

  memcpy(__data_chunk, (char *)payload + sizeof(bb_idata_msg_t), __data_size);

  __notifier->notify_of_data_change(__interface);
}


/** Reader has been added. */
void
BlackBoardInterfaceProxy::reader_added()
{
  ++__num_readers;
  __notifier->notify_of_reader_added(__interface->uid());
}

/** Reader has been removed. */
void
BlackBoardInterfaceProxy::reader_removed()
{
  if ( __num_readers > 0 ) {
    --__num_readers;
  }
  __notifier->notify_of_reader_removed(__interface);
}

/** Writer has been added. */
void
BlackBoardInterfaceProxy::writer_added()
{
  __has_writer = true;
  __notifier->notify_of_writer_added(__interface->uid());
}

/** Writer has been removed. */
void
BlackBoardInterfaceProxy::writer_removed()
{
  __has_writer = false;
  __notifier->notify_of_writer_removed(__interface);
}


/** Get instance serial of interface.
 * @return instance serial
 */
unsigned int
BlackBoardInterfaceProxy::serial()
{
  return __instance_serial;
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

void
BlackBoardInterfaceProxy::notify_of_data_change(const Interface *interface)
{
  // need to send write message
  size_t payload_size = sizeof(bb_idata_msg_t) + interface->datasize();
  void *payload = malloc(payload_size);
  bb_idata_msg_t *dm = (bb_idata_msg_t *)payload;
  dm->serial = interface->serial();
  dm->data_size = interface->datasize();
  memcpy((char *)payload + sizeof(bb_idata_msg_t), interface->datachunk(),
	 interface->datasize());

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(__clid, FAWKES_CID_BLACKBOARD,
							MSG_BB_DATA_CHANGED,
							payload, payload_size);
  __fnc->enqueue(omsg);
  omsg->unref();
}


/* MessageMediator */
unsigned int
BlackBoardInterfaceProxy::transmit(Message *message)
{
  return 0;
}
