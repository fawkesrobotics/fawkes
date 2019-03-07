
/***************************************************************************
 *  notifier.cpp - BlackBoard notifier
 *
 *  Created: Mon Mar 03 23:28:18 2008
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

#include <blackboard/blackboard.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <blackboard/internal/notifier.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/utils/lock_hashmap.h>
#include <core/utils/lock_hashset.h>
#include <interface/interface.h>
#include <logging/liblogger.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fnmatch.h>
#include <functional>

namespace fawkes {

/** @class BlackBoardNotifier <blackboard/internal/notifier.h>
 * BlackBoard notifier.
 * This class is used by the BlackBoard to notify listeners and observers
 * of changes. 
 *
 * @author Tim Niemueller
 */

/** Constructor. */
BlackBoardNotifier::BlackBoardNotifier()
{
	bbil_writer_events_ = 0;
	bbil_writer_mutex_  = new Mutex();

	bbil_reader_events_ = 0;
	bbil_reader_mutex_  = new Mutex();

	bbil_data_events_ = 0;
	bbil_data_mutex_  = new Mutex();

	bbil_messages_events_ = 0;
	bbil_messages_mutex_  = new Mutex();

	bbio_events_ = 0;
	bbio_mutex_  = new Mutex();
}

/** Destructor */
BlackBoardNotifier::~BlackBoardNotifier()
{
	delete bbil_writer_mutex_;
	delete bbil_reader_mutex_;
	delete bbil_data_mutex_;
	delete bbil_messages_mutex_;

	delete bbio_mutex_;
}

/** Register BB event listener.
 * @param listener BlackBoard event listener to register
 * @param flag concatenation of flags denoting which queue entries should be
 * processed
 */
void
BlackBoardNotifier::register_listener(BlackBoardInterfaceListener *    listener,
                                      BlackBoard::ListenerRegisterFlag flag)
{
	update_listener(listener, flag);
}

/** Update BB event listener.
 * @param listener BlackBoard event listener to update subscriptions of
 * @param flag concatenation of flags denoting which queue entries should be
 * processed
 */
void
BlackBoardNotifier::update_listener(BlackBoardInterfaceListener *    listener,
                                    BlackBoard::ListenerRegisterFlag flag)
{
	const BlackBoardInterfaceListener::InterfaceQueue &queue = listener->bbil_acquire_queue();

	BlackBoardInterfaceListener::InterfaceQueue::const_iterator i = queue.begin();

	for (i = queue.begin(); i != queue.end(); ++i) {
		switch (i->type) {
		case BlackBoardInterfaceListener::DATA:
			if (flag & BlackBoard::BBIL_FLAG_DATA) {
				proc_listener_maybe_queue(i->op,
				                          i->interface,
				                          listener,
				                          bbil_data_mutex_,
				                          bbil_data_events_,
				                          bbil_data_,
				                          bbil_data_queue_,
				                          "data");
			}
			break;
		case BlackBoardInterfaceListener::MESSAGES:
			if (flag & BlackBoard::BBIL_FLAG_MESSAGES) {
				proc_listener_maybe_queue(i->op,
				                          i->interface,
				                          listener,
				                          bbil_messages_mutex_,
				                          bbil_messages_events_,
				                          bbil_messages_,
				                          bbil_messages_queue_,
				                          "messages");
			}
			break;
		case BlackBoardInterfaceListener::READER:
			if (flag & BlackBoard::BBIL_FLAG_READER) {
				proc_listener_maybe_queue(i->op,
				                          i->interface,
				                          listener,
				                          bbil_reader_mutex_,
				                          bbil_reader_events_,
				                          bbil_reader_,
				                          bbil_reader_queue_,
				                          "reader");
			}
			break;
		case BlackBoardInterfaceListener::WRITER:
			if (flag & BlackBoard::BBIL_FLAG_WRITER) {
				proc_listener_maybe_queue(i->op,
				                          i->interface,
				                          listener,
				                          bbil_writer_mutex_,
				                          bbil_writer_events_,
				                          bbil_writer_,
				                          bbil_writer_queue_,
				                          "writer");
			}
			break;
		default: break;
		}
	}

	listener->bbil_release_queue(flag);
}

void
BlackBoardNotifier::proc_listener_maybe_queue(bool                         op,
                                              Interface *                  interface,
                                              BlackBoardInterfaceListener *listener,
                                              Mutex *                      mutex,
                                              unsigned int &               events,
                                              BBilMap &                    map,
                                              BBilQueue &                  queue,
                                              const char *                 hint)
{
	MutexLocker lock(mutex);
	if (events > 0) {
		LibLogger::log_warn("BlackBoardNotifier",
		                    "%s interface "
		                    "listener %s for %s events (queued)",
		                    op ? "Registering" : "Unregistering",
		                    listener->bbil_name(),
		                    hint);

		queue_listener(op, interface, listener, queue);
	} else {
		if (op) { // add
			add_listener(interface, listener, map);
		} else {
			remove_listener(interface, listener, map);
		}
	}
}

/** Unregister BB interface listener.
 * This will remove the given BlackBoard interface listener from any
 * event that it was previously registered for.
 * @param listener BlackBoard event listener to remove
 */
void
BlackBoardNotifier::unregister_listener(BlackBoardInterfaceListener *listener)
{
	const BlackBoardInterfaceListener::InterfaceMaps maps = listener->bbil_acquire_maps();

	BlackBoardInterfaceListener::InterfaceMap::const_iterator i;
	for (i = maps.data.begin(); i != maps.data.end(); ++i) {
		proc_listener_maybe_queue(false,
		                          i->second,
		                          listener,
		                          bbil_data_mutex_,
		                          bbil_data_events_,
		                          bbil_data_,
		                          bbil_data_queue_,
		                          "data");
	}

	for (i = maps.messages.begin(); i != maps.messages.end(); ++i) {
		proc_listener_maybe_queue(false,
		                          i->second,
		                          listener,
		                          bbil_messages_mutex_,
		                          bbil_messages_events_,
		                          bbil_messages_,
		                          bbil_messages_queue_,
		                          "messages");
	}

	for (i = maps.reader.begin(); i != maps.reader.end(); ++i) {
		proc_listener_maybe_queue(false,
		                          i->second,
		                          listener,
		                          bbil_reader_mutex_,
		                          bbil_reader_events_,
		                          bbil_reader_,
		                          bbil_reader_queue_,
		                          "reader");
	}

	for (i = maps.writer.begin(); i != maps.writer.end(); ++i) {
		proc_listener_maybe_queue(false,
		                          i->second,
		                          listener,
		                          bbil_writer_mutex_,
		                          bbil_writer_events_,
		                          bbil_writer_,
		                          bbil_writer_queue_,
		                          "writer");
	}

	listener->bbil_release_maps();
}

/** Add listener for specified map.
 * @param listener interface listener for events
 * @param im map of interfaces to listen for
 * @param ilmap internal map to add listener to
 */
void
BlackBoardNotifier::add_listener(Interface *                  interface,
                                 BlackBoardInterfaceListener *listener,
                                 BBilMap &                    ilmap)
{
	std::pair<BBilMap::iterator, BBilMap::iterator> ret = ilmap.equal_range(interface->uid());

	BBilMap::value_type v = std::make_pair(interface->uid(), listener);
	BBilMap::iterator   f = std::find(ret.first, ret.second, v);

	if (f == ret.second) {
		ilmap.insert(std::make_pair(interface->uid(), listener));
	}
}

void
BlackBoardNotifier::remove_listener(Interface *                  interface,
                                    BlackBoardInterfaceListener *listener,
                                    BBilMap &                    ilmap)
{
	std::pair<BBilMap::iterator, BBilMap::iterator> ret = ilmap.equal_range(interface->uid());
	for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
		if (j->second == listener) {
			ilmap.erase(j);
			break;
		}
	}
}

bool
BlackBoardNotifier::is_in_queue(bool                         op,
                                BBilQueue &                  queue,
                                const char *                 uid,
                                BlackBoardInterfaceListener *bbil)
{
	BBilQueue::iterator q;
	for (q = queue.begin(); q != queue.end(); ++q) {
		if ((q->op == op) && (q->uid == uid) && (q->listener == bbil)) {
			return true;
		}
	}
	return false;
}

void
BlackBoardNotifier::queue_listener(bool                         op,
                                   Interface *                  interface,
                                   BlackBoardInterfaceListener *listener,
                                   BBilQueue &                  queue)
{
	BBilQueueEntry qe = {op, interface->uid(), interface, listener};
	queue.push_back(qe);
}

/** Register BB interface observer.
 * @param observer BlackBoard interface observer to register
 */
void
BlackBoardNotifier::register_observer(BlackBoardInterfaceObserver *observer)
{
	bbio_mutex_->lock();
	if (bbio_events_ > 0) {
		bbio_queue_.push_back(std::make_pair(1, observer));
	} else {
		add_observer(observer, observer->bbio_get_observed_create(), bbio_created_);
		add_observer(observer, observer->bbio_get_observed_destroy(), bbio_destroyed_);
	}
	bbio_mutex_->unlock();
}

void
BlackBoardNotifier::add_observer(BlackBoardInterfaceObserver *                          observer,
                                 BlackBoardInterfaceObserver::ObservedInterfaceLockMap *its,
                                 BBioMap &                                              bbiomap)
{
	BlackBoardInterfaceObserver::ObservedInterfaceLockMapIterator i;
	its->lock();
	for (i = its->begin(); i != its->end(); ++i) {
		bbiomap[i->first].push_back(make_pair(observer, i->second));
	}
	its->unlock();
}

/** Remove observer from map.
 * @param iomap interface observer map to remove the observer from
 * @param observer observer to remove
 */
void
BlackBoardNotifier::remove_observer(BBioMap &iomap, BlackBoardInterfaceObserver *observer)
{
	BBioMapIterator i, tmp;

	i = iomap.begin();
	while (i != iomap.end()) {
		BBioListIterator j = i->second.begin();
		while (j != i->second.end()) {
			if (j->first == observer) {
				j = i->second.erase(j);
			} else {
				++j;
			}
		}
		if (i->second.empty()) {
			tmp = i;
			++i;
			iomap.erase(tmp);
		} else {
			++i;
		}
	}
}

/** Unregister BB interface observer.
 * This will remove the given BlackBoard event listener from any event that it was
 * previously registered for.
 * @param observer BlackBoard event listener to remove
 */
void
BlackBoardNotifier::unregister_observer(BlackBoardInterfaceObserver *observer)
{
	MutexLocker lock(bbio_mutex_);
	if (bbio_events_ > 0) {
		BBioQueueEntry      e = std::make_pair((unsigned int)0, observer);
		BBioQueue::iterator re;
		while ((re = find_if(bbio_queue_.begin(),
		                     bbio_queue_.end(),
		                     bind2nd(std::not_equal_to<BBioQueueEntry>(), e)))
		       != bbio_queue_.end()) {
			// if there is an entry in the register queue, remove it!
			if (re->second == observer) {
				bbio_queue_.erase(re);
			}
		}
		bbio_queue_.push_back(std::make_pair(0, observer));

	} else {
		remove_observer(bbio_created_, observer);
		remove_observer(bbio_destroyed_, observer);
	}
}

/** Notify that an interface has been created.
 * @param type type of the interface
 * @param id ID of the interface
 */
void
BlackBoardNotifier::notify_of_interface_created(const char *type, const char *id) throw()
{
	bbio_mutex_->lock();
	bbio_events_ += 1;
	bbio_mutex_->unlock();

	BBioMapIterator  lhmi;
	BBioListIterator i, l;
	for (lhmi = bbio_created_.begin(); lhmi != bbio_created_.end(); ++lhmi) {
		if (fnmatch(lhmi->first.c_str(), type, 0) != 0)
			continue;

		BBioList &list = lhmi->second;
		for (i = list.begin(); i != list.end(); ++i) {
			BlackBoardInterfaceObserver *bbio = i->first;
			for (std::list<std::string>::iterator pi = i->second.begin(); pi != i->second.end(); ++pi) {
				if (fnmatch(pi->c_str(), id, 0) == 0) {
					bbio->bb_interface_created(type, id);
					break;
				}
			}
		}
	}

	bbio_mutex_->lock();
	bbio_events_ -= 1;
	process_bbio_queue();
	bbio_mutex_->unlock();
}

/** Notify that an interface has been destroyed.
 * @param type type of the interface
 * @param id ID of the interface
 */
void
BlackBoardNotifier::notify_of_interface_destroyed(const char *type, const char *id) throw()
{
	bbio_mutex_->lock();
	bbio_events_ += 1;
	bbio_mutex_->unlock();

	BBioMapIterator  lhmi;
	BBioListIterator i, l;
	for (lhmi = bbio_destroyed_.begin(); lhmi != bbio_destroyed_.end(); ++lhmi) {
		if (fnmatch(lhmi->first.c_str(), type, 0) != 0)
			continue;

		BBioList &list = (*lhmi).second;
		for (i = list.begin(); i != list.end(); ++i) {
			BlackBoardInterfaceObserver *bbio = i->first;
			for (std::list<std::string>::iterator pi = i->second.begin(); pi != i->second.end(); ++pi) {
				if (fnmatch(pi->c_str(), id, 0) == 0) {
					bbio->bb_interface_destroyed(type, id);
					break;
				}
			}
		}
	}

	bbio_mutex_->lock();
	bbio_events_ -= 1;
	process_bbio_queue();
	bbio_mutex_->unlock();
}

void
BlackBoardNotifier::process_bbio_queue()
{
	if (!bbio_queue_.empty()) {
		if (bbio_events_ > 0) {
			return;
		} else {
			while (!bbio_queue_.empty()) {
				BBioQueueEntry &e = bbio_queue_.front();
				if (e.first) { // register
					add_observer(e.second, e.second->bbio_get_observed_create(), bbio_created_);
					add_observer(e.second, e.second->bbio_get_observed_destroy(), bbio_destroyed_);
				} else { // unregister
					remove_observer(bbio_created_, e.second);
					remove_observer(bbio_destroyed_, e.second);
				}
				bbio_queue_.pop_front();
			}
		}
	}
}

/** Notify that writer has been added.
 * @param interface the interface for which the event happened. It is not necessarily the
 * instance which caused the event, but it must have the same mem serial.
 * @param event_instance_serial the instance serial of the interface that caused the event
 * @see BlackBoardInterfaceListener::bb_interface_writer_added()
 */
void
BlackBoardNotifier::notify_of_writer_added(const Interface *interface,
                                           unsigned int     event_instance_serial) throw()
{
	bbil_writer_mutex_->lock();
	bbil_writer_events_ += 1;
	bbil_writer_mutex_->unlock();

	const char *                                    uid = interface->uid();
	std::pair<BBilMap::iterator, BBilMap::iterator> ret = bbil_writer_.equal_range(uid);
	for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
		BlackBoardInterfaceListener *bbil = j->second;
		if (!is_in_queue(/* remove op*/ false, bbil_writer_queue_, uid, bbil)) {
			Interface *bbil_iface = bbil->bbil_writer_interface(uid);
			if (bbil_iface != NULL) {
				bbil->bb_interface_writer_added(bbil_iface, event_instance_serial);
			} else {
				LibLogger::log_warn("BlackBoardNotifier",
				                    "BBIL[%s] registered for writer events "
				                    "(open) for '%s' but has no such interface",
				                    bbil->bbil_name(),
				                    uid);
			}
		}
	}

	bbil_writer_mutex_->lock();
	bbil_writer_events_ -= 1;
	process_writer_queue();
	bbil_writer_mutex_->unlock();
}

/** Notify that writer has been removed.
 * @param interface interface for which the writer has been removed
 * @param event_instance_serial instance serial of the interface that caused the event
 * @see BlackBoardInterfaceListener::bb_interface_writer_removed()
 */
void
BlackBoardNotifier::notify_of_writer_removed(const Interface *interface,
                                             unsigned int     event_instance_serial) throw()
{
	bbil_writer_mutex_->lock();
	bbil_writer_events_ += 1;
	bbil_writer_mutex_->unlock();

	const char *                                    uid = interface->uid();
	std::pair<BBilMap::iterator, BBilMap::iterator> ret = bbil_writer_.equal_range(uid);
	for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
		BlackBoardInterfaceListener *bbil = j->second;
		if (!is_in_queue(/* remove op*/ false, bbil_data_queue_, uid, bbil)) {
			Interface *bbil_iface = bbil->bbil_writer_interface(uid);
			if (bbil_iface != NULL) {
				bbil->bb_interface_writer_removed(bbil_iface, event_instance_serial);
			} else {
				LibLogger::log_warn("BlackBoardNotifier",
				                    "BBIL[%s] registered for writer events "
				                    "(close) for '%s' but has no such interface",
				                    bbil->bbil_name(),
				                    uid);
			}
		}
	}

	bbil_writer_mutex_->lock();
	bbil_writer_events_ -= 1;
	process_writer_queue();
	bbil_writer_mutex_->unlock();
}

void
BlackBoardNotifier::process_writer_queue()
{
	if (!bbil_writer_queue_.empty()) {
		if (bbil_writer_events_ > 0) {
			return;
		} else {
			while (!bbil_writer_queue_.empty()) {
				BBilQueueEntry &e = bbil_writer_queue_.front();
				if (e.op) { // register
					add_listener(e.interface, e.listener, bbil_writer_);
				} else { // unregister
					remove_listener(e.interface, e.listener, bbil_writer_);
				}
				bbil_writer_queue_.pop_front();
			}
		}
	}
}

/** Notify that reader has been added.
 * @param interface interface for which the reader has been added
 * @param event_instance_serial instance serial of the interface that caused the event
 * @see BlackBoardInterfaceListener::bb_interface_reader_added()
 */
void
BlackBoardNotifier::notify_of_reader_added(const Interface *interface,
                                           unsigned int     event_instance_serial) throw()
{
	bbil_reader_mutex_->lock();
	bbil_reader_events_ += 1;
	bbil_reader_mutex_->unlock();

	const char *                                    uid = interface->uid();
	std::pair<BBilMap::iterator, BBilMap::iterator> ret = bbil_reader_.equal_range(uid);
	for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
		BlackBoardInterfaceListener *bbil = j->second;
		if (!is_in_queue(/* remove op*/ false, bbil_reader_queue_, uid, bbil)) {
			Interface *bbil_iface = bbil->bbil_reader_interface(uid);
			if (bbil_iface != NULL) {
				bbil->bb_interface_reader_added(bbil_iface, event_instance_serial);
			} else {
				LibLogger::log_warn("BlackBoardNotifier",
				                    "BBIL[%s] registered for reader events "
				                    "(open) for '%s' but has no such interface",
				                    bbil->bbil_name(),
				                    uid);
			}
		}
	}

	bbil_reader_mutex_->lock();
	bbil_reader_events_ -= 1;
	process_reader_queue();
	bbil_reader_mutex_->unlock();
}

/** Notify that reader has been removed.
 * @param interface interface for which the reader has been removed
 * @param event_instance_serial instance serial of the interface that caused the event
 * @see BlackBoardInterfaceListener::bb_interface_reader_removed()
 */
void
BlackBoardNotifier::notify_of_reader_removed(const Interface *interface,
                                             unsigned int     event_instance_serial) throw()
{
	bbil_reader_mutex_->lock();
	bbil_reader_events_ += 1;
	bbil_reader_mutex_->unlock();

	const char *                                    uid = interface->uid();
	std::pair<BBilMap::iterator, BBilMap::iterator> ret = bbil_reader_.equal_range(uid);
	for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
		BlackBoardInterfaceListener *bbil = j->second;
		if (!is_in_queue(/* remove op*/ false, bbil_data_queue_, uid, bbil)) {
			Interface *bbil_iface = bbil->bbil_reader_interface(uid);
			if (bbil_iface != NULL) {
				bbil->bb_interface_reader_removed(bbil_iface, event_instance_serial);
			} else {
				LibLogger::log_warn("BlackBoardNotifier",
				                    "BBIL[%s] registered for reader events "
				                    "(close) for '%s' but has no such interface",
				                    bbil->bbil_name(),
				                    uid);
			}
		}
	}

	bbil_reader_mutex_->lock();
	bbil_reader_events_ -= 1;
	process_reader_queue();
	bbil_reader_mutex_->unlock();
}

void
BlackBoardNotifier::process_reader_queue()
{
	if (!bbil_reader_queue_.empty()) {
		if (bbil_reader_events_ > 0) {
			return;
		} else {
			while (!bbil_reader_queue_.empty()) {
				BBilQueueEntry &e = bbil_reader_queue_.front();
				if (e.op) { // register
					add_listener(e.interface, e.listener, bbil_reader_);
				} else { // unregister
					remove_listener(e.interface, e.listener, bbil_reader_);
				}
				bbil_reader_queue_.pop_front();
			}
		}
	}
}

/** Notify of data change.
 * Notify all subscribers of the given interface of a data change.
 * This also influences logging and sending data over the network so it is
 * mandatory to call this function! The interface base class write method does
 * that for you.
 * @param interface interface whose subscribers to notify
 * @see Interface::write()
 * @see BlackBoardInterfaceListener::bb_interface_data_changed()
 */
void
BlackBoardNotifier::notify_of_data_change(const Interface *interface)
{
	bbil_data_mutex_->lock();
	bbil_data_events_ += 1;
	bbil_data_mutex_->unlock();

	const char *                                    uid = interface->uid();
	std::pair<BBilMap::iterator, BBilMap::iterator> ret = bbil_data_.equal_range(uid);
	for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
		BlackBoardInterfaceListener *bbil = j->second;
		if (!is_in_queue(/* remove op*/ false, bbil_data_queue_, uid, bbil)) {
			Interface *bbil_iface = bbil->bbil_data_interface(uid);
			if (bbil_iface != NULL) {
				bbil->bb_interface_data_changed(bbil_iface);
			} else {
				LibLogger::log_warn("BlackBoardNotifier",
				                    "BBIL[%s] registered for data change events "
				                    "for '%s' but has no such interface",
				                    bbil->bbil_name(),
				                    uid);
			}
		}
	}

	bbil_data_mutex_->lock();
	bbil_data_events_ -= 1;
	if (!bbil_data_queue_.empty()) {
		if (bbil_data_events_ == 0) {
			while (!bbil_data_queue_.empty()) {
				BBilQueueEntry &e = bbil_data_queue_.front();
				if (e.op) { // register
					add_listener(e.interface, e.listener, bbil_data_);
				} else { // unregister
					remove_listener(e.interface, e.listener, bbil_data_);
				}
				bbil_data_queue_.pop_front();
			}
		}
	}
	bbil_data_mutex_->unlock();
}

/** Notify of message received
 * Notify all subscribers of the given interface of an incoming message
 * This also influences logging and sending data over the network so it is
 * mandatory to call this function! The interface base class write method does
 * that for you.
 * @param interface interface whose subscribers to notify
 * @param message message which is being received
 * @return false if any listener returned false, true otherwise
 * @see BlackBoardInterfaceListener::bb_interface_message_received()
 */
bool
BlackBoardNotifier::notify_of_message_received(const Interface *interface, Message *message)
{
	bbil_messages_mutex_->lock();
	bbil_messages_events_ += 1;
	bbil_messages_mutex_->unlock();

	bool enqueue = true;

	const char *                                    uid = interface->uid();
	std::pair<BBilMap::iterator, BBilMap::iterator> ret = bbil_messages_.equal_range(uid);
	for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
		BlackBoardInterfaceListener *bbil = j->second;
		if (!is_in_queue(/* remove op*/ false, bbil_messages_queue_, uid, bbil)) {
			Interface *bbil_iface = bbil->bbil_message_interface(uid);
			if (bbil_iface != NULL) {
				bool abort = !bbil->bb_interface_message_received(bbil_iface, message);
				if (abort) {
					enqueue = false;
					break;
				}
			} else {
				LibLogger::log_warn("BlackBoardNotifier",
				                    "BBIL[%s] registered for message events "
				                    "for '%s' but has no such interface",
				                    bbil->bbil_name(),
				                    uid);
			}
		}
	}

	bbil_messages_mutex_->lock();
	bbil_messages_events_ -= 1;
	if (!bbil_messages_queue_.empty()) {
		if (bbil_messages_events_ == 0) {
			while (!bbil_messages_queue_.empty()) {
				BBilQueueEntry &e = bbil_messages_queue_.front();
				if (e.op) { // register
					add_listener(e.interface, e.listener, bbil_messages_);
				} else { // unregister
					remove_listener(e.interface, e.listener, bbil_messages_);
				}
				bbil_messages_queue_.pop_front();
			}
		}
	}
	bbil_messages_mutex_->unlock();

	return enqueue;
}

} // end namespace fawkes
