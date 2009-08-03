
/***************************************************************************
 *  remote.h - Remote BlackBoard access via Fawkes network protocol
 *
 *  Created: Mon Mar 03 10:53:00 2008
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

#include <blackboard/remote.h>
#include <blackboard/exceptions.h>
#include <blackboard/net/messages.h>
#include <blackboard/net/ilist_content.h>
#include <blackboard/net/interface_proxy.h>
#include <blackboard/internal/notifier.h>
#include <blackboard/internal/instance_factory.h>

#include <interface/interface_info.h>

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/wait_condition.h>
#include <netcomm/fawkes/client.h>

#include <string>
#include <cstring>
#include <fnmatch.h>
#include <arpa/inet.h>

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

  __wait_mutex = new Mutex();
  __wait_cond  = new WaitCondition(__wait_mutex);

  __m = NULL;
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

  __fnc_owner = true;

  if ( ! __fnc->connected() ) {
    throw Exception("Cannot instantiate RemoteBlackBoard on unconnected client");
  }

  __fnc->register_handler(this, FAWKES_CID_BLACKBOARD);

  __mutex = new Mutex();
  __notifier = new BlackBoardNotifier();
  __instance_factory = new BlackBoardInstanceFactory();

  __wait_mutex = new Mutex();
  __wait_cond  = new WaitCondition(__wait_mutex);

  __m = NULL;
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

  delete __wait_cond;
  delete __wait_mutex;
}


bool
RemoteBlackBoard::is_alive() const throw()
{
  return __fnc->connected();
}


void
RemoteBlackBoard::reopen_interfaces()
{
  __proxies.lock();
  __ipit = __invalid_proxies.begin();
  while ( __ipit != __invalid_proxies.end() ) {
    try {
      Interface *iface = (*__ipit)->interface();
      open_interface(iface->type(), iface->id(), iface->is_writer(), iface);
      iface->set_validity(true);
      __ipit = __invalid_proxies.erase(__ipit);
    } catch (Exception &e) {
	  // we failed to re-establish validity for the given interface, bad luck
      ++__ipit;
    }
  }
  __proxies.unlock();
}

bool
RemoteBlackBoard::try_aliveness_restore() throw()
{
  bool rv = true;
  try {
    if ( ! __fnc->connected() ) {
      __fnc->connect();

      reopen_interfaces();
    }
  } catch (...) {
    rv = false;
  }
  return rv;
}


void
RemoteBlackBoard::open_interface(const char *type, const char *identifier,
				 bool writer, Interface *iface)
{
  if ( ! __fnc->connected() ) {
    throw Exception("Cannot instantiate remote interface, connection is dead");
  }

  bb_iopen_msg_t *om = (bb_iopen_msg_t *)calloc(1, sizeof(bb_iopen_msg_t));
  strncpy(om->type, type, __INTERFACE_TYPE_SIZE);
  strncpy(om->id, identifier, __INTERFACE_ID_SIZE);
  memcpy(om->hash, iface->hash(), __INTERFACE_HASH_SIZE);

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							writer ? MSG_BB_OPEN_FOR_WRITING : MSG_BB_OPEN_FOR_READING,
							om, sizeof(bb_iopen_msg_t));

  __wait_mutex->lock();
  __fnc->enqueue(omsg);
  omsg->unref();
  while (! __m ||
	 ((__m->msgid() != MSG_BB_OPEN_SUCCESS) &&
	  (__m->msgid() != MSG_BB_OPEN_FAILURE))) {
    if ( __m ) {
      __m->unref();
      __m = NULL;
    }
    __wait_cond->wait();
  }
  __wait_mutex->unlock();

  if ( __m->msgid() == MSG_BB_OPEN_SUCCESS ) {
    // We got the interface, create internal storage and prepare instance for return
    BlackBoardInterfaceProxy *proxy = new BlackBoardInterfaceProxy(__fnc, __m, __notifier,
								   iface, writer);
    __proxies[proxy->serial()] = proxy;
  } else if ( __m->msgid() == MSG_BB_OPEN_FAILURE ) {
    bb_iopenfail_msg_t *fm = __m->msg<bb_iopenfail_msg_t>();
    unsigned int error = ntohl(fm->errno);
    __m->unref();
    __m = NULL;
    if ( error == BB_ERR_WRITER_EXISTS ) {
      throw BlackBoardWriterActiveException(identifier, type);
    } else if ( error == BB_ERR_HASH_MISMATCH ) {
      throw Exception("Hash mismatch for interface %s:%s", type, identifier);
    } else if ( error == BB_ERR_UNKNOWN_TYPE ) {
      throw Exception("Type %s unknoen (%s:%s)", type, type, identifier);
    } else if ( error == BB_ERR_WRITER_EXISTS ) {
      throw BlackBoardWriterActiveException(identifier, type);
    } else {
      throw Exception("Could not open interface");
    }
  }

  __m->unref();
  __m = NULL;
}

Interface *
RemoteBlackBoard::open_interface(const char *type, const char *identifier, bool writer)
{
  if ( ! __fnc->connected() ) {
    throw Exception("Cannot instantiate remote interface, connection is dead");
  }

  Interface *iface = __instance_factory->new_interface_instance(type, identifier);
  try {
    open_interface(type, identifier, writer, iface);
  } catch (...) {
    __instance_factory->delete_interface_instance(iface);
    throw;
  }

  return iface;
}


Interface *
RemoteBlackBoard::open_for_reading(const char *type, const char *identifier)
{
  return open_interface(type, identifier, /* writer? */ false);
}


Interface *
RemoteBlackBoard::open_for_writing(const char *type, const char *identifier)
{
  return open_interface(type, identifier, /* writer? */ true);
}


std::list<Interface *>
RemoteBlackBoard::open_multiple_for_reading(const char *type, const char *id_pattern)
{
  std::list<Interface *> rv;

  InterfaceInfoList *infl = list_all();
  for (InterfaceInfoList::iterator i = infl->begin(); i != infl->end(); ++i) {
    if ((strncmp(type, i->type(), __INTERFACE_TYPE_SIZE) != 0) ||
	(fnmatch(id_pattern, i->id(), 0) == FNM_NOMATCH) ) {
      // type or ID prefix does not match, go on
      continue;
    }

    try {
      Interface *iface = open_for_reading((*i).type(), (*i).id());
      rv.push_back(iface);
    } catch (Exception &e) {
      for (std::list<Interface *>::iterator j = rv.begin(); j != rv.end(); ++j) {
	close(*j);
      }
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
    sm->serial = htonl(interface->serial());

    FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							  MSG_BB_CLOSE,
							  sm, sizeof(bb_iserial_msg_t));
    __fnc->enqueue(omsg);
    omsg->unref();
  }

  __instance_factory->delete_interface_instance(interface);
}


void
RemoteBlackBoard::register_listener(BlackBoardInterfaceListener *listener, unsigned int flags)
{
  __notifier->register_listener(listener, flags);
}


void
RemoteBlackBoard::unregister_listener(BlackBoardInterfaceListener *listener)
{
  __notifier->unregister_listener(listener);
}


void
RemoteBlackBoard::register_observer(BlackBoardInterfaceObserver *observer, unsigned int flags)
{
  __notifier->register_observer(observer, flags);
}


void
RemoteBlackBoard::unregister_observer(BlackBoardInterfaceObserver *observer)
{
  __notifier->unregister_observer(observer);
}


InterfaceInfoList *
RemoteBlackBoard::list_all()
{
  MutexLocker lock(__mutex);
  InterfaceInfoList *infl = new InterfaceInfoList();

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							MSG_BB_LIST_ALL);
  __wait_mutex->lock();
  __fnc->enqueue(omsg);
  omsg->unref();
  while (! __m ||
	 (__m->msgid() != MSG_BB_INTERFACE_LIST)) {
    if ( __m ) {
      __m->unref();
      __m = NULL;
    }
    __wait_cond->wait();
  }
  __wait_mutex->unlock();

  BlackBoardInterfaceListContent *bbilc = __m->msgc<BlackBoardInterfaceListContent>();
  while ( bbilc->has_next() ) {
    size_t iisize;
    bb_iinfo_msg_t *ii = bbilc->next(&iisize);
    infl->append(ii->type, ii->id, ii->hash,  ii->serial,
		 ii->has_writer, ii->num_readers);
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
	unsigned int serial = ntohl(((unsigned int *)m->payload())[0]);
	if ( __proxies.find(serial) != __proxies.end() ) {
	  __proxies[serial]->process_data_changed(m);
	}
      } else if (msgid == MSG_BB_INTERFACE_MESSAGE) {
	unsigned int serial = ntohl(((unsigned int *)m->payload())[0]);
	if ( __proxies.find(serial) != __proxies.end() ) {
	  __proxies[serial]->process_interface_message(m);
	}
      } else if (msgid == MSG_BB_READER_ADDED) {
	bb_ieventserial_msg_t *esm = m->msg<bb_ieventserial_msg_t>();
	if ( __proxies.find(ntohl(esm->serial)) != __proxies.end() ) {
	  __proxies[ntohl(esm->serial)]->reader_added(ntohl(esm->event_serial));
	}
      } else if (msgid == MSG_BB_READER_REMOVED) {
	bb_ieventserial_msg_t *esm = m->msg<bb_ieventserial_msg_t>();
	if ( __proxies.find(ntohl(esm->serial)) != __proxies.end() ) {
	  __proxies[ntohl(esm->serial)]->reader_removed(ntohl(esm->event_serial));
	}
      } else if (msgid == MSG_BB_WRITER_ADDED) {
	bb_ieventserial_msg_t *esm = m->msg<bb_ieventserial_msg_t>();
	if ( __proxies.find(ntohl(esm->serial)) != __proxies.end() ) {
	  __proxies[ntohl(esm->serial)]->writer_added(ntohl(esm->event_serial));
	}
      } else if (msgid == MSG_BB_WRITER_REMOVED) {
	bb_ieventserial_msg_t *esm = m->msg<bb_ieventserial_msg_t>();
	if ( __proxies.find(ntohl(esm->serial)) != __proxies.end() ) {
	  __proxies[ntohl(esm->serial)]->writer_removed(ntohl(esm->event_serial));
	}
      } else {
	__wait_mutex->stopby();
	__m = m;
	__m->ref();
	__wait_cond->wake_all();
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
    __invalid_proxies.push_back(__pit->second);
  }
  __proxies.clear();
  __proxies.unlock();
}


void
RemoteBlackBoard::connection_established(unsigned int id) throw()
{
}

} // end namespace fawkes
