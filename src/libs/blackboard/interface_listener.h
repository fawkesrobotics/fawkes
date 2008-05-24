 
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

#include <core/utils/lock_map.h>
#include <utils/misc/string_compare.h>
#include <string>

namespace fawkes {

class Interface;
class Message;

class BlackBoardInterfaceListener
{
 friend class BlackBoardNotifier;

 public:
  /** Type for lockable interface maps. */
 typedef  LockMap<std::string, Interface *> InterfaceLockMap;
 /** Iterator for InterfaceLockMap */
 typedef  InterfaceLockMap::iterator   InterfaceLockMapIterator;


  BlackBoardInterfaceListener(const char *name_format, ...);
  virtual ~BlackBoardInterfaceListener();

  const char * bbil_name() const;

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

  InterfaceLockMap *      bbil_data_interfaces() throw();
  InterfaceLockMap *      bbil_message_interfaces() throw();
  InterfaceLockMap *      bbil_reader_interfaces() throw();
  InterfaceLockMap *      bbil_writer_interfaces() throw();

  Interface * bbil_data_interface(const char *iuid) throw();
  Interface * bbil_message_interface(const char *iuid) throw();
  Interface * bbil_reader_interface(const char *iuid) throw();
  Interface * bbil_writer_interface(const char *iuid) throw();

 private:
  InterfaceLockMap         __bbil_data_interfaces;
  InterfaceLockMap         __bbil_message_interfaces;
  InterfaceLockMap         __bbil_reader_interfaces;
  InterfaceLockMap         __bbil_writer_interfaces;

  InterfaceLockMapIterator __bbil_ii;

  char *__name;
};

} // end namespace fawkes

#endif
