 
/***************************************************************************
 *  interface_listener.h - BlackBoard event listener
 *
 *  Created: Wed Nov 07 23:55:53 2007 (Saw Ella for the first time)
 *  Copyright  2007-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_INTERFACE_LISTENER_H_
#define __BLACKBOARD_INTERFACE_LISTENER_H_

#include <core/utils/lock_hashmap.h>
#include <core/utils/lock_hashset.h>
#include <utils/misc/string_compare.h>

class Interface;
class Message;

class BlackBoardInterfaceListener
{
 friend class BlackBoardNotifier;

 public:
  /** Type for lockable interface hash maps. */
  typedef  LockHashMap<char *, Interface *,
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 2)
                       std::tr1::hash<char *>,
#else
                       __gnu_cxx::hash<char *>,
#endif
                       StringEquality >            InterfaceLockHashMap;

  /** Type for iterator of lockable interface hash maps. */
  typedef  LockHashMap<char *, Interface *,
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 2)
                       std::tr1::hash<char *>,
#else
                       __gnu_cxx::hash<char *>,
#endif
                       StringEquality >::iterator  InterfaceLockHashMapIterator;


  BlackBoardInterfaceListener();
  virtual ~BlackBoardInterfaceListener();

  virtual void bb_interface_data_changed(Interface *interface) throw();
  virtual bool bb_interface_message_received(Interface *interface, Message *message) throw();
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

  InterfaceLockHashMap *      bbil_data_interfaces() throw();
  InterfaceLockHashMap *      bbil_message_interfaces() throw();
  InterfaceLockHashMap *      bbil_reader_interfaces() throw();
  InterfaceLockHashMap *      bbil_writer_interfaces() throw();

  Interface * bbil_data_interface(const char *iuid) throw();
  Interface * bbil_message_interface(const char *iuid) throw();
  Interface * bbil_reader_interface(const char *iuid) throw();
  Interface * bbil_writer_interface(const char *iuid) throw();

 private:
  InterfaceLockHashMap       __bbil_data_interfaces;
  InterfaceLockHashMap       __bbil_message_interfaces;
  InterfaceLockHashMap       __bbil_reader_interfaces;
  InterfaceLockHashMap       __bbil_writer_interfaces;

  InterfaceLockHashMapIterator __bbil_ii;
};


#endif
