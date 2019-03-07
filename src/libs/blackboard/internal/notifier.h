
/***************************************************************************
 *  notifier.h - BlackBoard notifier
 *
 *  Created: Mon Mar 03 23:25:57 2008
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

#ifndef _BLACKBOARD_NOTIFIER_H_
#define _BLACKBOARD_NOTIFIER_H_

#include <blackboard/blackboard.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/utils/rwlock_map.h>

#include <list>
#include <string>
#include <utility>

namespace fawkes {

class Interface;
class Message;
class Mutex;

class BlackBoardNotifier
{
public:
	BlackBoardNotifier();
	virtual ~BlackBoardNotifier();

	void register_listener(BlackBoardInterfaceListener *    listener,
	                       BlackBoard::ListenerRegisterFlag flag);
	void update_listener(BlackBoardInterfaceListener *    listener,
	                     BlackBoard::ListenerRegisterFlag flag);
	void unregister_listener(BlackBoardInterfaceListener *listener);

	void register_observer(BlackBoardInterfaceObserver *observer);
	void unregister_observer(BlackBoardInterfaceObserver *observer);

	void notify_of_data_change(const Interface *interface);
	bool notify_of_message_received(const Interface *interface, Message *message);
	void notify_of_interface_created(const char *type, const char *id) throw();
	void notify_of_interface_destroyed(const char *type, const char *id) throw();
	void notify_of_writer_added(const Interface *interface,
	                            unsigned int     event_instance_serial) throw();
	void notify_of_writer_removed(const Interface *interface,
	                              unsigned int     event_instance_serial) throw();
	void notify_of_reader_added(const Interface *interface,
	                            unsigned int     event_instance_serial) throw();
	void notify_of_reader_removed(const Interface *interface,
	                              unsigned int     event_instance_serial) throw();

private:
	/// @cond INTERNALS
	typedef struct
	{
		bool                         op;
		std::string                  uid;
		Interface *                  interface;
		BlackBoardInterfaceListener *listener;
	} BBilQueueEntry;
	/// @endcond INTERNALS
	typedef std::list<BBilQueueEntry> BBilQueue;

	typedef std::multimap<std::string, BlackBoardInterfaceListener *>        BBilMap;
	typedef std::pair<BlackBoardInterfaceObserver *, std::list<std::string>> BBioPair;
	typedef std::list<BBioPair>                                              BBioList;
	typedef std::map<std::string, BBioList>                                  BBioMap;

	// Type to observer, add flags, 0 to remove
	typedef std::pair<unsigned int, BlackBoardInterfaceObserver *> BBioQueueEntry;
	typedef std::list<BBioQueueEntry>                              BBioQueue;

	typedef BBilMap::iterator BBilMapIterator;

	typedef BBioList::iterator BBioListIterator;
	typedef BBioMap::iterator  BBioMapIterator;

	void proc_listener_maybe_queue(bool                         op,
	                               Interface *                  interface,
	                               BlackBoardInterfaceListener *listener,
	                               Mutex *                      mutex,
	                               unsigned int &               events,
	                               BBilMap &                    map,
	                               BBilQueue &                  queue,
	                               const char *                 hint);

	void add_listener(Interface *interface, BlackBoardInterfaceListener *listener, BBilMap &ilmap);
	void remove_listener(Interface *interface, BlackBoardInterfaceListener *listener, BBilMap &ilmap);
	void queue_listener(bool                         op,
	                    Interface *                  interface,
	                    BlackBoardInterfaceListener *listener,
	                    BBilQueue &                  queue);

	void add_observer(BlackBoardInterfaceObserver *                          observer,
	                  BlackBoardInterfaceObserver::ObservedInterfaceLockMap *its,
	                  BBioMap &                                              bbiomap);

	void remove_observer(BBioMap &iomap, BlackBoardInterfaceObserver *observer);

	void process_writer_queue();
	void process_reader_queue();
	void process_data_queue();
	void process_bbio_queue();

	bool is_in_queue(bool op, BBilQueue &queue, const char *uid, BlackBoardInterfaceListener *bbil);

	BBilMap bbil_data_;
	BBilMap bbil_reader_;
	BBilMap bbil_writer_;
	BBilMap bbil_messages_;

	Mutex *   bbil_unregister_mutex_;
	BBilQueue bbil_unregister_queue_;

	Mutex *      bbil_writer_mutex_;
	unsigned int bbil_writer_events_;
	BBilQueue    bbil_writer_queue_;

	Mutex *      bbil_reader_mutex_;
	unsigned int bbil_reader_events_;
	BBilQueue    bbil_reader_queue_;

	Mutex *      bbil_data_mutex_;
	unsigned int bbil_data_events_;
	BBilQueue    bbil_data_queue_;

	Mutex *      bbil_messages_mutex_;
	unsigned int bbil_messages_events_;
	BBilQueue    bbil_messages_queue_;

	BBioMap bbio_created_;
	BBioMap bbio_destroyed_;

	Mutex *      bbio_mutex_;
	unsigned int bbio_events_;
	BBioQueue    bbio_queue_;
};

} // end namespace fawkes

#endif
