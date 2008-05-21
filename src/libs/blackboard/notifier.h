 
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

#include <core/utils/lock_map.h>

#include <list>
#include <string>

namespace fawkes {

class Interface;
class Message;
class BlackBoardInterfaceListener;
class BlackBoardInterfaceObserver;

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
  typedef LockMap< std::string, BBilList >            BBilLockMap;

  typedef std::list< BlackBoardInterfaceObserver * >  BBioList;
  typedef LockMap< std::string, BBioList >            BBioLockMap;

  typedef std::list< BlackBoardInterfaceListener * >::iterator BBilListIterator;
  typedef BBilLockMap::iterator BBilLockMapIterator;

  typedef std::list< BlackBoardInterfaceObserver * >::iterator BBioListIterator;
  typedef BBioLockMap::iterator BBioLockMapIterator;

  void remove_listener(BBilLockMap &ifmap, BlackBoardInterfaceListener *listener);
  void remove_observer(BBioLockMap &iomap, BlackBoardInterfaceObserver *observer);

  BBilLockMap __bbil_data;
  BBilLockMap __bbil_messages;
  BBilLockMap __bbil_reader;
  BBilLockMap __bbil_writer;

  BBioLockMap __bbio_created;
  BBioLockMap __bbio_destroyed;

};

} // end namespace fawkes

#endif
