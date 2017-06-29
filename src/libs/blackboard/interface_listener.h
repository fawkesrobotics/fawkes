 
/***************************************************************************
 *  interface_listener.h - BlackBoard event listener
 *
 *  Created: Wed Nov 07 23:55:53 2007 (Saw Ella for the first time)
 *  Copyright  2007-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_INTERFACE_LISTENER_H_
#define __BLACKBOARD_INTERFACE_LISTENER_H_

#include <blackboard/blackboard.h>
#include <core/utils/lock_queue.h>
#include <utils/misc/string_compare.h>
#include <string>
#include <map>
#include <list>

namespace fawkes {

class Interface;
class Message;
class BlackBoardNotifier;

class BlackBoardInterfaceListener
{
 friend BlackBoardNotifier;

 public:
 /** Queue entry type. */
 typedef enum {
   DATA = 0,		///< Data changed event entry
   MESSAGES = 1,	///< Message received event entry
   READER = 2,		///< Reader event entry
   WRITER = 3		///< Writer event entry
 } QueueEntryType;

 /** Queue entry type. */
 typedef struct {
   QueueEntryType type;		///< What type this entry concerns
   bool           op;		///< true to add, false to remove
   Interface *    interface;	///< interface this entry concerns
 } QueueEntry;

  /** Queue of additions/removal of interfaces. */
 typedef std::list<QueueEntry> InterfaceQueue;

  /** Map of currently active event subscriptions. */
 typedef std::map<std::string, Interface *> InterfaceMap;

 /** Structure to hold maps for active subscriptions. */
 typedef struct {
   InterfaceMap  data;		///< Data event subscriptions
   InterfaceMap  messages;	///< Message received event subscriptions
   InterfaceMap  reader;	///< Reader event subscriptions
   InterfaceMap  writer;	///< Writer event subscriptions
 } InterfaceMaps;

  BlackBoardInterfaceListener(const char *name_format, ...);
  virtual ~BlackBoardInterfaceListener();

  const char * bbil_name() const;

  virtual void bb_interface_data_changed(Interface *interface) throw();
  virtual bool bb_interface_message_received(Interface *interface,
                                             Message *message) throw();
  virtual void bb_interface_writer_added(Interface *interface,
					 unsigned int instance_serial) throw();
  virtual void bb_interface_writer_removed(Interface *interface,
					   unsigned int instance_serial) throw();
  virtual void bb_interface_reader_added(Interface *interface,
					 unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(Interface *interface,
					   unsigned int instance_serial) throw();

 protected:
  void bbil_add_data_interface(Interface *interface);
  void bbil_add_message_interface(Interface *interface);
  void bbil_add_reader_interface(Interface *interface);
  void bbil_add_writer_interface(Interface *interface);

  void bbil_remove_data_interface(Interface *interface);
  void bbil_remove_message_interface(Interface *interface);
  void bbil_remove_reader_interface(Interface *interface);
  void bbil_remove_writer_interface(Interface *interface);

  Interface * bbil_data_interface(const char *iuid) throw();
  Interface * bbil_message_interface(const char *iuid) throw();
  Interface * bbil_reader_interface(const char *iuid) throw();
  Interface * bbil_writer_interface(const char *iuid) throw();


 private:
  void bbil_queue_add(QueueEntryType type, bool op,
                      InterfaceMap &not_in_map,
                      Interface *interface, const char *hint);
  Interface * bbil_find_interface(const char *iuid, InterfaceMap &map);

  const InterfaceQueue &  bbil_acquire_queue() throw();
  void bbil_release_queue(BlackBoard::ListenerRegisterFlag flag) throw();

  const InterfaceMaps & bbil_acquire_maps() throw();
  void bbil_release_maps() throw();


 private:
  Mutex *__bbil_queue_mutex;
  Mutex *__bbil_maps_mutex;

  InterfaceMaps  __bbil_maps;
  InterfaceQueue __bbil_queue;

  char *__name;
};

} // end namespace fawkes

#endif
