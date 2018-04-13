
/***************************************************************************
 *  remote.h - Remote BlackBoard access via Fawkes network protocol
 *
 *  Created: Mon Mar 03 10:53:00 2008
 *  Copyright  2006-2015  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/wait_condition.h>
#include <netcomm/fawkes/client.h>
#include <utils/time/time.h>

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
  __instance_factory = new BlackBoardInstanceFactory();

  __wait_mutex = new Mutex();
  __wait_cond  = new WaitCondition(__wait_mutex);

  __inbound_thread = NULL;
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
  __instance_factory = new BlackBoardInstanceFactory();

  __wait_mutex = new Mutex();
  __wait_cond  = new WaitCondition(__wait_mutex);

  __inbound_thread = NULL;
  __m = NULL;
}


/** Destructor. */
RemoteBlackBoard::~RemoteBlackBoard()
{
  __fnc->deregister_handler(FAWKES_CID_BLACKBOARD);
  delete __mutex;
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
      open_interface(iface->type(), iface->id(), iface->owner(), iface->is_writer(), iface);
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
RemoteBlackBoard::open_interface(const char *type, const char *identifier, const char *owner,
				 bool writer, Interface *iface)
{
  if ( ! __fnc->connected() ) {
    throw Exception("Cannot instantiate remote interface, connection is dead");
  }

  __mutex->lock();
  if (__inbound_thread != NULL &&
      Thread::current_thread() &&
      strcmp(Thread::current_thread()->name(), __inbound_thread) == 0)
  {
    throw Exception("Cannot call open_interface() from inbound handler");
  }
  __mutex->unlock();

  bb_iopen_msg_t *om = (bb_iopen_msg_t *)calloc(1, sizeof(bb_iopen_msg_t));
  strncpy(om->type, type, __INTERFACE_TYPE_SIZE-1);
  strncpy(om->id, identifier, __INTERFACE_ID_SIZE-1);
  memcpy(om->hash, iface->hash(), __INTERFACE_HASH_SIZE);

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							writer ? MSG_BB_OPEN_FOR_WRITING : MSG_BB_OPEN_FOR_READING,
							om, sizeof(bb_iopen_msg_t));

  __wait_mutex->lock();
  __fnc->enqueue(omsg);
  while (is_alive() &&
	 (! __m ||
	   ((__m->msgid() != MSG_BB_OPEN_SUCCESS) &&
	   (__m->msgid() != MSG_BB_OPEN_FAILURE))))
  {
    if ( __m ) {
      __m->unref();
      __m = NULL;
    }
    __wait_cond->wait();
  }
  __wait_mutex->unlock();

  if (!is_alive()) {
    throw Exception("Connection died while trying to open %s::%s",
		    type, identifier);
  }

  if ( __m->msgid() == MSG_BB_OPEN_SUCCESS ) {
    // We got the interface, create internal storage and prepare instance for return
    BlackBoardInterfaceProxy *proxy = new BlackBoardInterfaceProxy(__fnc, __m, __notifier,
								   iface, writer);
    __proxies[proxy->serial()] = proxy;
  } else if ( __m->msgid() == MSG_BB_OPEN_FAILURE ) {
    bb_iopenfail_msg_t *fm = __m->msg<bb_iopenfail_msg_t>();
    unsigned int error = ntohl(fm->error_code);
    __m->unref();
    __m = NULL;
    if ( error == BB_ERR_WRITER_EXISTS ) {
      throw BlackBoardWriterActiveException(identifier, type);
    } else if ( error == BB_ERR_HASH_MISMATCH ) {
      throw Exception("Hash mismatch for interface %s:%s", type, identifier);
    } else if ( error == BB_ERR_UNKNOWN_TYPE ) {
      throw Exception("Type %s unknown (%s::%s)", type, type, identifier);
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
RemoteBlackBoard::open_interface(const char *type, const char *identifier, const char *owner, bool writer)
{
  if ( ! __fnc->connected() ) {
    throw Exception("Cannot instantiate remote interface, connection is dead");
  }

  Interface *iface = __instance_factory->new_interface_instance(type, identifier);
  try {
    open_interface(type, identifier, owner, writer, iface);
  } catch (Exception &e) {
    __instance_factory->delete_interface_instance(iface);
    throw;
  }

  return iface;
}


Interface *
RemoteBlackBoard::open_for_reading(const char *type, const char *identifier, const char *owner)
{
  return open_interface(type, identifier, owner, /* writer? */ false);
}


Interface *
RemoteBlackBoard::open_for_writing(const char *type, const char *identifier, const char *owner)
{
  return open_interface(type, identifier, owner, /* writer? */ true);
}


std::list<Interface *>
RemoteBlackBoard::open_multiple_for_reading(const char *type_pattern,
					    const char *id_pattern, const char *owner)
{
  std::list<Interface *> rv;

  InterfaceInfoList *infl = list_all();
  for (InterfaceInfoList::iterator i = infl->begin(); i != infl->end(); ++i) {
    // ensure 0-termination
    char type[__INTERFACE_TYPE_SIZE + 1];
    char id[__INTERFACE_ID_SIZE + 1];
    type[__INTERFACE_TYPE_SIZE] = 0;
    id[__INTERFACE_TYPE_SIZE] = 0;
    strncpy(type, i->type(), __INTERFACE_TYPE_SIZE);
    strncpy(id, i->id(), __INTERFACE_ID_SIZE);

    if ((fnmatch(type_pattern, type, 0) == FNM_NOMATCH) ||
	(fnmatch(id_pattern, id, 0) == FNM_NOMATCH) ) {
      // type or ID prefix does not match, go on
      continue;
    }

    try {
      Interface *iface = open_for_reading((*i).type(), (*i).id(), owner);
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
  }

  __instance_factory->delete_interface_instance(interface);
}


InterfaceInfoList *
RemoteBlackBoard::list_all()
{
  __mutex->lock();
  if (__inbound_thread != NULL &&
      strcmp(Thread::current_thread()->name(), __inbound_thread) == 0)
  {
    throw Exception("Cannot call list_all() from inbound handler");
  }
  __mutex->unlock();

  InterfaceInfoList *infl = new InterfaceInfoList();

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							MSG_BB_LIST_ALL);
  __wait_mutex->lock();
  __fnc->enqueue(omsg);
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
    bool has_writer = ii->writer_readers & htonl(0x80000000);
    unsigned int num_readers = ntohl(ii->writer_readers & htonl(0x7FFFFFFF));
    infl->append(ii->type, ii->id, ii->hash,  ntohl(ii->serial),
		 has_writer, num_readers, std::list<std::string>(), std::string(),
		 fawkes::Time(ii->timestamp_sec, ii->timestamp_usec));
  }

  __m->unref();
  __m = NULL;

  return infl;
}


InterfaceInfoList *
RemoteBlackBoard::list(const char *type_pattern, const char *id_pattern)
{
  __mutex->lock();
  if (__inbound_thread != NULL &&
      strcmp(Thread::current_thread()->name(), __inbound_thread) == 0)
  {
    throw Exception("Cannot call list() from inbound handler");
  }
  __mutex->unlock();

  InterfaceInfoList *infl = new InterfaceInfoList();

  bb_ilistreq_msg_t *om =
    (bb_ilistreq_msg_t *)calloc(1, sizeof(bb_ilistreq_msg_t));
  strncpy(om->type_pattern, type_pattern, __INTERFACE_TYPE_SIZE-1);
  strncpy(om->id_pattern, id_pattern, __INTERFACE_ID_SIZE-1);

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							MSG_BB_LIST,
							om,
							sizeof(bb_ilistreq_msg_t));

  __wait_mutex->lock();
  __fnc->enqueue(omsg);
  while (! __m ||
	 (__m->msgid() != MSG_BB_INTERFACE_LIST)) {
    if ( __m ) {
      __m->unref();
      __m = NULL;
    }
    __wait_cond->wait();
  }
  __wait_mutex->unlock();

  BlackBoardInterfaceListContent *bbilc =
    __m->msgc<BlackBoardInterfaceListContent>();
  while ( bbilc->has_next() ) {
    size_t iisize;
    bb_iinfo_msg_t *ii = bbilc->next(&iisize);
    bool has_writer = ii->writer_readers & htonl(0x80000000);
    unsigned int num_readers = ntohl(ii->writer_readers & htonl(0x7FFFFFFF));
    infl->append(ii->type, ii->id, ii->hash, ntohl(ii->serial),
		 has_writer, num_readers, std::list<std::string>(), std::string(),
		 fawkes::Time(ii->timestamp_sec, ii->timestamp_usec));
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
  __mutex->lock();
  __inbound_thread = Thread::current_thread()->name();
  __mutex->unlock();

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
      } else if (msgid == MSG_BB_INTERFACE_CREATED) {
	bb_ievent_msg_t *em = m->msg<bb_ievent_msg_t>();
	__notifier->notify_of_interface_created(em->type, em->id);
      } else if (msgid == MSG_BB_INTERFACE_DESTROYED) {
	bb_ievent_msg_t *em = m->msg<bb_ievent_msg_t>();
	__notifier->notify_of_interface_destroyed(em->type, em->id);
      } else {
	__wait_mutex->lock();
	__m = m;
	__m->ref();
	__wait_cond->wake_all();
	__wait_mutex->unlock();
      }
    } catch (Exception &e) {
      // Bam, you're dead. Ok, not now, we just ignore that this shit happened...
    }
  }

  __mutex->lock();
  __inbound_thread = NULL;
  __mutex->unlock();
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
  __wait_cond->wake_all();
}


void
RemoteBlackBoard::connection_established(unsigned int id) throw()
{
}

} // end namespace fawkes
