 
/***************************************************************************
 *  interface_observer.h - BlackBoard interface observer
 *
 *  Created: Fri Jan 25 18:19:00 2008 (Just back from Hacking with Fawkes talk)
 *  Copyright  2007-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __BLACKBOARD_INTERFACE_OBSERVER_H_
#define __BLACKBOARD_INTERFACE_OBSERVER_H_

#include <core/utils/lock_hashset.h>
#include <utils/misc/string_compare.h>

class BlackBoardInterfaceObserver
{

 friend class BlackBoardNotifier;

 public:
  BlackBoardInterfaceObserver();
  virtual ~BlackBoardInterfaceObserver();

  virtual void bb_interface_created(const char *type, const char *id) throw();
  virtual void bb_interface_destroyed(const char *type, const char *id) throw();

 protected:
  void bbio_add_interface_create_type(const char *type) throw();
  void bbio_add_interface_destroy_type(const char *type) throw();


  /** Type for lockable interface type hash sets. */
  typedef  LockHashSet<char *,
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 2)
                       std::tr1::hash<char *>,
#else
                       __gnu_cxx::hash<char *>,
#endif
                       StringEquality >            InterfaceTypeLockHashSet;

  /** Type for iterator of lockable interface type hash sets. */
  typedef  LockHashSet<char *,
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 2)
                       std::tr1::hash<char *>,
#else
                       __gnu_cxx::hash<char *>,
#endif
                       StringEquality >::iterator  InterfaceTypeLockHashSetIterator;

  InterfaceTypeLockHashSet *  bbio_interface_create_types() throw();
  InterfaceTypeLockHashSet *  bbio_interface_destroy_types() throw();

 private:
  InterfaceTypeLockHashSet         __bbio_interface_create_types;
  InterfaceTypeLockHashSet         __bbio_interface_destroy_types;
  InterfaceTypeLockHashSetIterator __bbio_iti;
};


#endif
