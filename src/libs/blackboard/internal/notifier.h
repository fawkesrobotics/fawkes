 
/***************************************************************************
 *  notifier.h - BlackBoard notifier
 *
 *  Created: Mon Mar 03 23:25:57 2008
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

#ifndef __BLACKBOARD_NOTIFIER_H_
#define __BLACKBOARD_NOTIFIER_H_

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
class WaitCondition;

class BlackBoardNotifier
{
 public:
  BlackBoardNotifier();
  virtual ~BlackBoardNotifier();

  void register_listener(BlackBoardInterfaceListener *listener, unsigned int flags);
  void unregister_listener(BlackBoardInterfaceListener *listener);

  void register_observer(BlackBoardInterfaceObserver *observer, unsigned int flags);
  void unregister_observer(BlackBoardInterfaceObserver *observer);

  void notify_of_data_change(const Interface *interface);
  bool notify_of_message_received(const Interface *interface, Message *message);
  void notify_of_interface_created(const char *type, const char *id) throw();
  void notify_of_interface_destroyed(const char *type, const char *id) throw();
  void notify_of_writer_added(const Interface *interface,
			      unsigned int event_instance_serial) throw();
  void notify_of_writer_removed(const Interface *interface,
				unsigned int event_instance_serial) throw();
  void notify_of_reader_added(const Interface *interface,
			      unsigned int event_instance_serial) throw();
  void notify_of_reader_removed(const Interface *interface,
				unsigned int event_instance_serial) throw();

 private:
  typedef std::list< BlackBoardInterfaceListener * >  BBilList;
  typedef std::map< std::string, BBilList >            BBilMap;

  typedef std::pair< bool, BlackBoardInterfaceListener *> BBilQueueEntry;
  typedef std::list< BBilQueueEntry > BBilQueue;

  typedef std::map< std::string, BlackBoardInterfaceListener * > BBilMessageLockMap;
  typedef std::map< std::string, BlackBoardInterfaceListener * >::iterator BBilMessageLockMapIterator;

  typedef std::pair<BlackBoardInterfaceObserver *, std::list<std::string> > BBioPair;
  typedef std::list< BBioPair>                   BBioList;
  typedef std::map< std::string, BBioList >     BBioMap;

  // Type to observer, add flags, 0 to remove
  typedef std::pair< unsigned int, BlackBoardInterfaceObserver *> BBioQueueEntry;
  typedef std::list< BBioQueueEntry > BBioQueue;

  typedef BBilList::iterator    BBilListIterator;
  typedef BBilMap::iterator     BBilMapIterator;

  typedef BBioList::iterator    BBioListIterator;
  typedef BBioMap::iterator     BBioMapIterator;

  void add_listener(BlackBoardInterfaceListener *listener,
		    BlackBoardInterfaceListener::InterfaceLockMap *im,
		    BBilMap &ilmap);

  void remove_listener(BlackBoardInterfaceListener *listener,
		       Mutex *mutex, unsigned int events,
		       BBilQueue &queue, BBilMap &ilmap);
  void remove_listener(BBilMap &ifmap, BlackBoardInterfaceListener *listener);
  void remove_message_listener(BlackBoardInterfaceListener *listener);
  void remove_message_listener_map(BlackBoardInterfaceListener *listener);

  void add_observer(BlackBoardInterfaceObserver *observer,
		    BlackBoardInterfaceObserver::ObservedInterfaceLockMap *its,
		    BBioMap &bbiomap);

  void remove_observer(BBioMap &iomap, BlackBoardInterfaceObserver *observer);

  void process_writer_queue();
  void process_reader_queue();
  void process_data_queue();
  void process_bbio_queue();

  BBilMap __bbil_data;
  BBilMap __bbil_reader;
  BBilMap __bbil_writer;
  BBilMessageLockMap __bbil_messages;

  Mutex         *__bbil_unregister_mutex;
  WaitCondition *__bbil_unregister_waitcond;
  BBilQueue      __bbil_unregister_queue;

  Mutex         *__bbil_writer_mutex;
  WaitCondition *__bbil_writer_waitcond;
  unsigned int   __bbil_writer_events;
  BBilQueue      __bbil_writer_queue;

  Mutex         *__bbil_reader_mutex;
  WaitCondition *__bbil_reader_waitcond;
  unsigned int   __bbil_reader_events;
  BBilQueue      __bbil_reader_queue;

  Mutex         *__bbil_data_mutex;
  WaitCondition *__bbil_data_waitcond;
  unsigned int   __bbil_data_events;
  BBilQueue      __bbil_data_queue;

  Mutex         *__bbil_messages_mutex;
  WaitCondition *__bbil_messages_waitcond;
  unsigned int   __bbil_messages_events;
  BBilQueue      __bbil_messages_queue;

  BBioMap        __bbio_created;
  BBioMap        __bbio_destroyed;

  Mutex         *__bbio_mutex;
  WaitCondition *__bbio_waitcond;
  unsigned int   __bbio_events;
  BBioQueue      __bbio_queue;

};

} // end namespace fawkes

#endif
