
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
  fnc_ = client;
  fnc_owner_ = false;

  if ( ! fnc_->connected() ) {
    throw Exception("Cannot instantiate RemoteBlackBoard on unconnected client");
  }

  fnc_->register_handler(this, FAWKES_CID_BLACKBOARD);

  mutex_ = new Mutex();
  instance_factory_ = new BlackBoardInstanceFactory();

  wait_mutex_ = new Mutex();
  wait_cond_  = new WaitCondition(wait_mutex_);

  inbound_thread_ = NULL;
  m_ = NULL;
}


/** Constructor.
 * This will internall create a fawkes network client that is used to communicate
 * with the remote BlackBoard.
 * @param hostname hostname to connect to
 * @param port port to connect to
 */
RemoteBlackBoard::RemoteBlackBoard(const char *hostname, unsigned short int port)
{
  fnc_ = new FawkesNetworkClient(hostname, port);
  try {
    fnc_->connect();
  } catch (Exception &e) {
    delete fnc_;
    throw;
  }

  fnc_owner_ = true;

  if ( ! fnc_->connected() ) {
    throw Exception("Cannot instantiate RemoteBlackBoard on unconnected client");
  }

  fnc_->register_handler(this, FAWKES_CID_BLACKBOARD);

  mutex_ = new Mutex();
  instance_factory_ = new BlackBoardInstanceFactory();

  wait_mutex_ = new Mutex();
  wait_cond_  = new WaitCondition(wait_mutex_);

  inbound_thread_ = NULL;
  m_ = NULL;
}


/** Destructor. */
RemoteBlackBoard::~RemoteBlackBoard()
{
  fnc_->deregister_handler(FAWKES_CID_BLACKBOARD);
  delete mutex_;
  delete instance_factory_;

  for ( pit_ = proxies_.begin(); pit_ != proxies_.end(); ++pit_) {
    delete pit_->second;
  }

  if (fnc_owner_) {
    fnc_->disconnect();
    delete fnc_;
  }

  delete wait_cond_;
  delete wait_mutex_;
}


bool
RemoteBlackBoard::is_alive() const throw()
{
  return fnc_->connected();
}


void
RemoteBlackBoard::reopen_interfaces()
{
  proxies_.lock();
  ipit_ = invalid_proxies_.begin();
  while ( ipit_ != invalid_proxies_.end() ) {
    try {
      Interface *iface = (*ipit_)->interface();
      open_interface(iface->type(), iface->id(), iface->owner(), iface->is_writer(), iface);
      iface->set_validity(true);
      ipit_ = invalid_proxies_.erase(ipit_);
    } catch (Exception &e) {
	  // we failed to re-establish validity for the given interface, bad luck
      ++ipit_;
    }
  }
  proxies_.unlock();
}

bool
RemoteBlackBoard::try_aliveness_restore() throw()
{
  bool rv = true;
  try {
    if ( ! fnc_->connected() ) {
      fnc_->connect();

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
  if ( ! fnc_->connected() ) {
    throw Exception("Cannot instantiate remote interface, connection is dead");
  }

  mutex_->lock();
  if (inbound_thread_ != NULL &&
      Thread::current_thread() &&
      strcmp(Thread::current_thread()->name(), inbound_thread_) == 0)
  {
    throw Exception("Cannot call open_interface() from inbound handler");
  }
  mutex_->unlock();

  bb_iopen_msg_t *om = (bb_iopen_msg_t *)calloc(1, sizeof(bb_iopen_msg_t));
  strncpy(om->type, type, INTERFACE_TYPE_SIZE_-1);
  strncpy(om->id, identifier, INTERFACE_ID_SIZE_-1);
  memcpy(om->hash, iface->hash(), INTERFACE_HASH_SIZE_);

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							writer ? MSG_BB_OPEN_FOR_WRITING : MSG_BB_OPEN_FOR_READING,
							om, sizeof(bb_iopen_msg_t));

  wait_mutex_->lock();
  fnc_->enqueue(omsg);
  while (is_alive() &&
	 (! m_ ||
	   ((m_->msgid() != MSG_BB_OPEN_SUCCESS) &&
	   (m_->msgid() != MSG_BB_OPEN_FAILURE))))
  {
    if ( m_ ) {
      m_->unref();
      m_ = NULL;
    }
    wait_cond_->wait();
  }
  wait_mutex_->unlock();

  if (!is_alive()) {
    throw Exception("Connection died while trying to open %s::%s",
		    type, identifier);
  }

  if ( m_->msgid() == MSG_BB_OPEN_SUCCESS ) {
    // We got the interface, create internal storage and prepare instance for return
    BlackBoardInterfaceProxy *proxy = new BlackBoardInterfaceProxy(fnc_, m_, notifier_,
								   iface, writer);
    proxies_[proxy->serial()] = proxy;
  } else if ( m_->msgid() == MSG_BB_OPEN_FAILURE ) {
    bb_iopenfail_msg_t *fm = m_->msg<bb_iopenfail_msg_t>();
    unsigned int error = ntohl(fm->error_code);
    m_->unref();
    m_ = NULL;
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

  m_->unref();
  m_ = NULL;
}

Interface *
RemoteBlackBoard::open_interface(const char *type, const char *identifier, const char *owner, bool writer)
{
  if ( ! fnc_->connected() ) {
    throw Exception("Cannot instantiate remote interface, connection is dead");
  }

  Interface *iface = instance_factory_->new_interface_instance(type, identifier);
  try {
    open_interface(type, identifier, owner, writer, iface);
  } catch (Exception &e) {
    instance_factory_->delete_interface_instance(iface);
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
    char type[INTERFACE_TYPE_SIZE_ + 1];
    char id[INTERFACE_ID_SIZE_ + 1];
    type[INTERFACE_TYPE_SIZE_] = 0;
    id[INTERFACE_TYPE_SIZE_] = 0;
    strncpy(type, i->type(), INTERFACE_TYPE_SIZE_);
    strncpy(id, i->id(), INTERFACE_ID_SIZE_);

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

  if ( proxies_.find(serial) != proxies_.end() ) {
    delete proxies_[serial];
    proxies_.erase(serial);
  }

  if ( fnc_->connected() ) {
    // We cannot "officially" close it, if we are disconnected it cannot be used anyway
    bb_iserial_msg_t *sm = (bb_iserial_msg_t *)calloc(1, sizeof(bb_iserial_msg_t));
    sm->serial = htonl(interface->serial());

    FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							  MSG_BB_CLOSE,
							  sm, sizeof(bb_iserial_msg_t));
    fnc_->enqueue(omsg);
  }

  instance_factory_->delete_interface_instance(interface);
}


InterfaceInfoList *
RemoteBlackBoard::list_all()
{
  mutex_->lock();
  if (inbound_thread_ != NULL &&
      strcmp(Thread::current_thread()->name(), inbound_thread_) == 0)
  {
    throw Exception("Cannot call list_all() from inbound handler");
  }
  mutex_->unlock();

  InterfaceInfoList *infl = new InterfaceInfoList();

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							MSG_BB_LIST_ALL);
  wait_mutex_->lock();
  fnc_->enqueue(omsg);
  while (! m_ ||
	 (m_->msgid() != MSG_BB_INTERFACE_LIST)) {
    if ( m_ ) {
      m_->unref();
      m_ = NULL;
    }
    wait_cond_->wait();
  }
  wait_mutex_->unlock();

  BlackBoardInterfaceListContent *bbilc = m_->msgc<BlackBoardInterfaceListContent>();
  while ( bbilc->has_next() ) {
    size_t iisize;
    bb_iinfo_msg_t *ii = bbilc->next(&iisize);
    bool has_writer = ii->writer_readers & htonl(0x80000000);
    unsigned int num_readers = ntohl(ii->writer_readers & htonl(0x7FFFFFFF));
    infl->append(ii->type, ii->id, ii->hash,  ntohl(ii->serial),
		 has_writer, num_readers, std::list<std::string>(), std::string(),
		 fawkes::Time(ii->timestamp_sec, ii->timestamp_usec));
  }

  m_->unref();
  m_ = NULL;

  return infl;
}


InterfaceInfoList *
RemoteBlackBoard::list(const char *type_pattern, const char *id_pattern)
{
  mutex_->lock();
  if (inbound_thread_ != NULL &&
      strcmp(Thread::current_thread()->name(), inbound_thread_) == 0)
  {
    throw Exception("Cannot call list() from inbound handler");
  }
  mutex_->unlock();

  InterfaceInfoList *infl = new InterfaceInfoList();

  bb_ilistreq_msg_t *om =
    (bb_ilistreq_msg_t *)calloc(1, sizeof(bb_ilistreq_msg_t));
  strncpy(om->type_pattern, type_pattern, INTERFACE_TYPE_SIZE_-1);
  strncpy(om->id_pattern, id_pattern, INTERFACE_ID_SIZE_-1);

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_BLACKBOARD,
							MSG_BB_LIST,
							om,
							sizeof(bb_ilistreq_msg_t));

  wait_mutex_->lock();
  fnc_->enqueue(omsg);
  while (! m_ ||
	 (m_->msgid() != MSG_BB_INTERFACE_LIST)) {
    if ( m_ ) {
      m_->unref();
      m_ = NULL;
    }
    wait_cond_->wait();
  }
  wait_mutex_->unlock();

  BlackBoardInterfaceListContent *bbilc =
    m_->msgc<BlackBoardInterfaceListContent>();
  while ( bbilc->has_next() ) {
    size_t iisize;
    bb_iinfo_msg_t *ii = bbilc->next(&iisize);
    bool has_writer = ii->writer_readers & htonl(0x80000000);
    unsigned int num_readers = ntohl(ii->writer_readers & htonl(0x7FFFFFFF));
    infl->append(ii->type, ii->id, ii->hash, ntohl(ii->serial),
		 has_writer, num_readers, std::list<std::string>(), std::string(),
		 fawkes::Time(ii->timestamp_sec, ii->timestamp_usec));
  }

  m_->unref();
  m_ = NULL;

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
  mutex_->lock();
  inbound_thread_ = Thread::current_thread()->name();
  mutex_->unlock();

  if ( m->cid() == FAWKES_CID_BLACKBOARD ) {
    unsigned int msgid = m->msgid();
    try {
      if ( msgid == MSG_BB_DATA_CHANGED ) {
	unsigned int serial = ntohl(((unsigned int *)m->payload())[0]);
	if ( proxies_.find(serial) != proxies_.end() ) {
	  proxies_[serial]->process_data_changed(m);
	}
      } else if (msgid == MSG_BB_INTERFACE_MESSAGE) {
	unsigned int serial = ntohl(((unsigned int *)m->payload())[0]);
	if ( proxies_.find(serial) != proxies_.end() ) {
	  proxies_[serial]->process_interface_message(m);
	}
      } else if (msgid == MSG_BB_READER_ADDED) {
	bb_ieventserial_msg_t *esm = m->msg<bb_ieventserial_msg_t>();
	if ( proxies_.find(ntohl(esm->serial)) != proxies_.end() ) {
	  proxies_[ntohl(esm->serial)]->reader_added(ntohl(esm->event_serial));
	}
      } else if (msgid == MSG_BB_READER_REMOVED) {
	bb_ieventserial_msg_t *esm = m->msg<bb_ieventserial_msg_t>();
	if ( proxies_.find(ntohl(esm->serial)) != proxies_.end() ) {
	  proxies_[ntohl(esm->serial)]->reader_removed(ntohl(esm->event_serial));
	}
      } else if (msgid == MSG_BB_WRITER_ADDED) {
	bb_ieventserial_msg_t *esm = m->msg<bb_ieventserial_msg_t>();
	if ( proxies_.find(ntohl(esm->serial)) != proxies_.end() ) {
	  proxies_[ntohl(esm->serial)]->writer_added(ntohl(esm->event_serial));
	}
      } else if (msgid == MSG_BB_WRITER_REMOVED) {
	bb_ieventserial_msg_t *esm = m->msg<bb_ieventserial_msg_t>();
	if ( proxies_.find(ntohl(esm->serial)) != proxies_.end() ) {
	  proxies_[ntohl(esm->serial)]->writer_removed(ntohl(esm->event_serial));
	}
      } else if (msgid == MSG_BB_INTERFACE_CREATED) {
	bb_ievent_msg_t *em = m->msg<bb_ievent_msg_t>();
	notifier_->notify_of_interface_created(em->type, em->id);
      } else if (msgid == MSG_BB_INTERFACE_DESTROYED) {
	bb_ievent_msg_t *em = m->msg<bb_ievent_msg_t>();
	notifier_->notify_of_interface_destroyed(em->type, em->id);
      } else {
	wait_mutex_->lock();
	m_ = m;
	m_->ref();
	wait_cond_->wake_all();
	wait_mutex_->unlock();
      }
    } catch (Exception &e) {
      // Bam, you're dead. Ok, not now, we just ignore that this shit happened...
    }
  }

  mutex_->lock();
  inbound_thread_ = NULL;
  mutex_->unlock();
}


void
RemoteBlackBoard::connection_died(unsigned int id) throw()
{
  // mark all assigned interfaces as invalid
  proxies_.lock();
  for (pit_ = proxies_.begin(); pit_ != proxies_.end(); ++pit_) {
    pit_->second->interface()->set_validity(false);
    invalid_proxies_.push_back(pit_->second);
  }
  proxies_.clear();
  proxies_.unlock();
  wait_cond_->wake_all();
}


void
RemoteBlackBoard::connection_established(unsigned int id) throw()
{
}

} // end namespace fawkes
