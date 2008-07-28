
/***************************************************************************
 *  remote.h - Remote BlackBoard access via Fawkes network protocol
 *
 *  Created: Mon Mar 03 10:53:00 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/remote.h>
#include <blackboard/net_messages.h>
#include <blackboard/net_ilist_content.h>
#include <blackboard/notifier.h>
#include <blackboard/instance_factory.h>
#include <blackboard/interface_proxy.h>
#include <blackboard/exceptions.h>

#include <interface/interface_info.h>

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <netcomm/fawkes/client.h>

#include <string>
#include <cstring>

namespace fawkes {

/** @class RemoteBlackBoard <blackboard/remote.h>
 * Remote BlackBoard.
 * This class implements the access to a remote BlackBoard using the Fawkes
 * network protocol.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param client Fawkes network client to use.
 */
RemoteBlackBoard::RemoteBlackBoard(FawkesNetworkClient *client)
{
  __fnc = client;
  __fnc_owner = false;

  if ( ! __fnc->connected() ) {
    throw Exception("Cannot instantiate RemoteBlackBoard on unconnected client");
  }

  __fnc->register_handler(this, FAWKES_CID_BLACKBOARD);

  __mutex = new Mutex();
  __notifier = new BlackBoardNotifier();
  __instance_factory = new BlackBoardInstanceFactory();
}


/** Constructor.
 * This will internall create a fawkes network client that is used to communicate
 * with the remote BlackBoard.
 * @param hostname hostname to connect to
 * @param port port to connect to
 */
RemoteBlackBoard::RemoteBlackBoard(const char *hostname, unsigned short int port)
{
  __fnc = new FawkesNetworkClient(hostname, port);
  try {
    __fnc->connect();
  } catch (Exception &e) {
    delete __fnc;
    throw;
  }

  __fnc->wait_connection_established();
  __fnc_owner = true;

  if ( ! __fnc->connected() ) {
    throw Exception("Cannot instantiate RemoteBlackBoard on unconnected client");
  }

  __fnc->register_handler(this, FAWKES_CID_BLACKBOARD);

  __mutex = new Mutex();
  __notifier = new BlackBoardNotifier();
  __instance_factory = new BlackBoardInstanceFactory();
}


/** Destructor. */
RemoteBlackBoard::~RemoteBlackBoard()
{
  __fnc->deregister_handler(FAWKES_CID_BLACKBOARD);
  delete __mutex;
  delete __notifier;
  delete __instance_factory;

  for ( __pit = __proxies.begin(); __pit != __proxies.end(); ++__pit) {
    delete __pit->second;
  }

  if (__fnc_owner) {
    __fnc->disconnect();
    delete __fnc;
  }
}


bool
RemoteBlackBoard::is_alive() const throw()
{
  return __fnc->connected();
}


Interface *
RemoteBlackBoard::open_interface(const char *type, const char *identifier, bool writer)
{
  if ( ! __fnc->connected() ) {
    throw Exception("Cannot instantiate remote interface, connection is dead");
  }

  Interface *iface = __instance_factory->new_interface_instance(type, identifier);

  bb_iopen_msg_t *om = (bb_iopen_msg_t *)calloc(1, sizeof(bb_iopen_msg_t));
  strncpy(om->type, type, __INTERFACE_TYPE_SIZE);
  strncpy(om->id, identifier, __INTERFACE_ID_SIZE);
  memcpy(om->hash, iface->hash(), __INTERFACE_HASH_SIZE);

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							writer ? MSG_BB_OPEN_FOR_WRITING : MSG_BB_OPEN_FOR_READING,
							om, sizeof(bb_iopen_msg_t));
  __fnc->enqueue_and_wait(omsg);

  if ( !__m ) {
    __instance_factory->delete_interface_instance(iface);
    throw Exception("No message or invalid message ID");
  } else if ( __m->msgid() == MSG_BB_OPEN_SUCCESS ) {
    // We got the interface, create internal storage and prepare instance for return
    BlackBoardInterfaceProxy *proxy = new BlackBoardInterfaceProxy(__fnc, __m, __notifier,
								   iface, writer);
    __proxies[proxy->serial()] = proxy;
  } else if ( __m->msgid() == MSG_BB_OPEN_FAILURE ) {
    bb_iopenfail_msg_t *fm = __m->msg<bb_iopenfail_msg_t>();
    unsigned int error = fm->errno;
    __m->unref();
    __m = NULL;
    __instance_factory->delete_interface_instance(iface);
    if ( error == BB_ERR_WRITER_EXISTS ) {
      throw BlackBoardWriterActiveException(identifier, type);
    } else {
      throw Exception("Could not open inteface");
    }
  } else {
    __m->unref();
    __m = NULL;
    __instance_factory->delete_interface_instance(iface);
    throw Exception("Unexpected message received");
  }

  __m->unref();
  __m = NULL;

  return iface;
}


/** Open interface for reading.
 * This will create a new interface instance of the given type. The result can be
 * casted to the appropriate type.
 * @param type type of the interface
 * @param identifier identifier of the interface
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 */
Interface *
RemoteBlackBoard::open_for_reading(const char *type, const char *identifier)
{
  return open_interface(type, identifier, /* writer? */ false);
}


/** Open interface for writing.
 * This will create a new interface instance of the given type. The result can be
 * casted to the appropriate type. This will only succeed if there is not already
 * a writer for the given interface type/id!
 * @param type type of the interface
 * @param identifier identifier of the interface
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception BlackBoardWriterActiveException thrown if there is already a writing
 * instance with the same type/id
 */
Interface *
RemoteBlackBoard::open_for_writing(const char *type, const char *identifier)
{
  return open_interface(type, identifier, /* writer? */ true);
}


/** Open all interfaces of the given type for reading.
 * This will create interface instances for all currently registered interfaces of
 * the given type. The result can be casted to the appropriate type.
 * @param type type of the interface
 * @param id_prefix if set only interfaces whose ids have this prefix are returned
 * @return list of new fully initialized interface instances of requested type. The
 * is allocated using new and you have to free it using delete after you are done
 * with it!
 */
std::list<Interface *> *
RemoteBlackBoard::open_all_of_type_for_reading(const char *type, const char *id_prefix)
{
  std::list<Interface *> *rv = new std::list<Interface *>();

  unsigned int prefix_len = (id_prefix != NULL) ? strlen(id_prefix) : 0;

  InterfaceInfoList *infl = list_all();
  for (InterfaceInfoList::iterator i = infl->begin(); i != infl->end(); ++i) {
    if ((strncmp(type, (*i).type(), __INTERFACE_TYPE_SIZE) != 0) ||
	(prefix_len > strlen((*i).id())) ||
	(strncmp(id_prefix, (*i).id(), prefix_len) != 0) ) {
      // type or ID prefix does not match, go on
      continue;
    }

    try {
      Interface *iface = open_for_reading((*i).type(), (*i).id());
      rv->push_back(iface);
    } catch (Exception &e) {
      for (std::list<Interface *>::iterator j = rv->begin(); j != rv->end(); ++j) {
	close(*j);
      }
      delete rv;
      throw;
    }
  }

  return rv;
}


/** Close interface.
 * @param interface interface to close
 */
void
RemoteBlackBoard::close(Interface *interface)
{
  if ( interface == NULL )  return;

  unsigned int serial = interface->serial();

  if ( __proxies.find(serial) != __proxies.end() ) {
    delete __proxies[serial];
    __proxies.erase(serial);
  }

  if ( __fnc->connected() ) {
    // We cannot "officially" close it, if we are disconnected it cannot be used anyway
    bb_iserial_msg_t *sm = (bb_iserial_msg_t *)calloc(1, sizeof(bb_iserial_msg_t));
    sm->serial = interface->serial();

    FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							  MSG_BB_CLOSE,
							  sm, sizeof(bb_iserial_msg_t));
    __fnc->enqueue(omsg);
    omsg->unref();
  }

  __instance_factory->delete_interface_instance(interface);
}


/** Register BB event listener.
 * @param listener BlackBoard event listener to register
 * @param flags an or'ed combination of BBIL_FLAG_DATA, BBIL_FLAG_READER, BBIL_FLAG_WRITER
 * and BBIL_FLAG_INTERFACE. Only for the given types the event listener is registered.
 * BBIL_FLAG_ALL can be supplied to register for all events.
 */
void
RemoteBlackBoard::register_listener(BlackBoardInterfaceListener *listener, unsigned int flags)
{
  __notifier->register_listener(listener, flags);
}


/** Unregister BB interface listener.
 * This will remove the given BlackBoard interface listener from any event that it was
 * previously registered for.
 * @param listener BlackBoard event listener to remove
 */
void
RemoteBlackBoard::unregister_listener(BlackBoardInterfaceListener *listener)
{
  __notifier->unregister_listener(listener);
}


/** Register BB interface observer.
 * @param observer BlackBoard interface observer to register
 * @param flags an or'ed combination of BBIO_FLAG_CREATED, BBIO_FLAG_DESTROYED
 */
void
RemoteBlackBoard::register_observer(BlackBoardInterfaceObserver *observer, unsigned int flags)
{
  __notifier->register_observer(observer, flags);
}


/** Unregister BB interface observer.
 * This will remove the given BlackBoard event listener from any event that it was
 * previously registered for.
 * @param observer BlackBoard event listener to remove
 */
void
RemoteBlackBoard::unregister_observer(BlackBoardInterfaceObserver *observer)
{
  __notifier->unregister_observer(observer);
}


/** Get list of interfaces.
 * @return list of interfaces
 */
InterfaceInfoList *
RemoteBlackBoard::list_all()
{
  MutexLocker lock(__mutex);
  InterfaceInfoList *infl = new InterfaceInfoList();

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							MSG_BB_LIST_ALL);
  __fnc->enqueue_and_wait(omsg);

  if ( !__m || (__m->msgid() != MSG_BB_INTERFACE_LIST) ) {
    throw Exception("No message or invalid message ID");
  }

  BlackBoardInterfaceListContent *bbilc = __m->msgc<BlackBoardInterfaceListContent>();
  while ( bbilc->has_next() ) {
    size_t iisize;
    bb_iinfo_msg_t *ii = bbilc->next(&iisize);
    infl->append(ii->type, ii->id, ii->hash, ii->has_writer, ii->num_readers, ii->serial);
  }

  __m->unref();
  __m = NULL;

  return infl;
}


/** We are no longer registered in Fawkes network client.
 * Ignored.
 * @param id the id of the calling client
 */
void
RemoteBlackBoard::deregistered(unsigned int id) throw()
{
}


void
RemoteBlackBoard::inbound_received(FawkesNetworkMessage *m,
				   unsigned int id) throw()
{
  if ( m->cid() == FAWKES_CID_BLACKBOARD ) {
    unsigned int msgid = m->msgid();
    try {
      if ( msgid == MSG_BB_DATA_CHANGED ) {
	unsigned int *serial = (unsigned int *)m->payload();
	if ( __proxies.find(*serial) != __proxies.end() ) {
	  __proxies[*serial]->process_data_changed(m);
	}
      } else if (msgid == MSG_BB_INTERFACE_MESSAGE) {
	unsigned int *serial = (unsigned int *)m->payload();
	if ( __proxies.find(*serial) != __proxies.end() ) {
	  __proxies[*serial]->process_interface_message(m);
	}
      } else if (msgid == MSG_BB_READER_ADDED) {
	bb_ieventserial_msg_t *esm = m->msg<bb_ieventserial_msg_t>();
	if ( __proxies.find(esm->serial) != __proxies.end() ) {
	  __proxies[esm->serial]->reader_added(esm->event_serial);
	}
      } else if (msgid == MSG_BB_READER_REMOVED) {
	bb_ieventserial_msg_t *esm = m->msg<bb_ieventserial_msg_t>();
	if ( __proxies.find(esm->serial) != __proxies.end() ) {
	  __proxies[esm->serial]->reader_removed(esm->event_serial);
	}
      } else if (msgid == MSG_BB_WRITER_ADDED) {
	bb_ieventserial_msg_t *esm = m->msg<bb_ieventserial_msg_t>();
	if ( __proxies.find(esm->serial) != __proxies.end() ) {
	  __proxies[esm->serial]->writer_added(esm->event_serial);
	}
      } else if (msgid == MSG_BB_WRITER_REMOVED) {
	bb_ieventserial_msg_t *esm = m->msg<bb_ieventserial_msg_t>();
	if ( __proxies.find(esm->serial) != __proxies.end() ) {
	  __proxies[esm->serial]->writer_removed(esm->event_serial);
	}
      } else {
	__m = m;
	__m->ref();
      }
    } catch (Exception &e) {
      // Bam, you're dead. Ok, not now, we just ignore that this shit happened...
    }
  }
}


void
RemoteBlackBoard::connection_died(unsigned int id) throw()
{
  // mark all assigned interfaces as invalid
  __proxies.lock();
  for (__pit = __proxies.begin(); __pit != __proxies.end(); ++__pit) {
    __pit->second->interface()->set_validity(false);
  }
  __proxies.unlock();
}


void
RemoteBlackBoard::connection_established(unsigned int id) throw()
{
}

} // end namespace fawkes
