 
/***************************************************************************
 *  event_listener.h - BlackBoard event listener
 *
 *  Created: Wed Nov 07 23:55:53 2007 (Saw Stella for the first time)
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __BLACKBOARD_EVENT_LISTENER_H_
#define __BLACKBOARD_EVENT_LISTENER_H_

#include <core/utils/lock_hashmap.h>
#include <core/utils/lock_hashset.h>
#include <utils/misc/string_compare.h>

class Interface;

class BlackBoardEventListener
{

 friend class BlackBoardInterfaceManager;

 public:
  /** Type for locked interface hash maps. */
  typedef  LockHashMap<char *, Interface *,
                       __gnu_cxx::hash<char *>, StringEquality > InterfaceLockHashMap;

  /** Type for iterator of locked interface hash maps. */
  typedef  LockHashMap<char *, Interface *,  __gnu_cxx::hash<char *>,
                       StringEquality >::iterator InterfaceLockHashMapIterator;

  /** Type for locked interface type hash sets. */
  typedef  LockHashSet<char *,  __gnu_cxx::hash<char *>,
                       StringEquality > InterfaceTypeLockHashSet;

  /** Type for iterator of locked interface type hash sets. */
  typedef  LockHashSet<char *,  __gnu_cxx::hash<char *>,
                       StringEquality >::iterator InterfaceTypeLockHashSetIterator;

  BlackBoardEventListener();
  virtual ~BlackBoardEventListener();

  virtual void bb_data_changed(Interface *interface) throw();
  virtual void bb_interface_created(const char *type, const char *id) throw();
  virtual void bb_interface_writer_added(Interface *interface) throw();
  virtual void bb_interface_writer_removed(Interface *interface) throw();
  virtual void bb_interface_reader_added(Interface *interface) throw();
  virtual void bb_interface_reader_removed(Interface *interface) throw();

 protected:
  void bbel_add_data_interface(Interface *interface);
  void bbel_add_reader_interface(Interface *interface);
  void bbel_add_writer_interface(Interface *interface);
  void bbel_add_interface_create_type(const char *type) throw();

  InterfaceLockHashMap *      bbel_data_interfaces() throw();
  InterfaceLockHashMap *      bbel_reader_interfaces() throw();
  InterfaceLockHashMap *      bbel_writer_interfaces() throw();
  InterfaceTypeLockHashSet *  bbel_interface_create_types() throw();

  Interface * bbel_data_interface(const char *iuid) throw();
  Interface * bbel_reader_interface(const char *iuid) throw();
  Interface * bbel_writer_interface(const char *iuid) throw();

 private:
  InterfaceLockHashMap       __bbel_data_interfaces;
  InterfaceLockHashMap       __bbel_reader_interfaces;
  InterfaceLockHashMap       __bbel_writer_interfaces;

  InterfaceTypeLockHashSet   __bbel_interface_create_types;

  InterfaceLockHashMapIterator __bbel_ii;
  InterfaceTypeLockHashSetIterator __bbel_iti;
};


#endif
