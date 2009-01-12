
/***************************************************************************
 *  blackboard.h - BlackBoard Interface
 *
 *  Created: Sat Sep 16 17:09:15 2006 (on train to Cologne)
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

#ifndef __BLACKBOARD_BLACKBOARD_H_
#define __BLACKBOARD_BLACKBOARD_H_

#include <core/exceptions/software.h>

#include <list>
#include <typeinfo>

namespace fawkes {

class BlackBoardInterfaceManager;
class BlackBoardMemoryManager;
class BlackBoardMessageManager;
class BlackBoardNetworkHandler;
class BlackBoardNotifier;
class Interface;
class InterfaceInfoList;
class BlackBoardInterfaceListener;
class BlackBoardInterfaceObserver;
class FawkesNetworkHub;

class BlackBoard
{
 public:
  virtual ~BlackBoard();

  virtual Interface *  open_for_reading(const char *interface_type, const char *identifier) = 0;
  virtual Interface *  open_for_writing(const char *interface_type, const char *identifier) = 0;
  virtual void         close(Interface *interface) = 0;

  virtual InterfaceInfoList *  list_all() = 0;
  virtual bool                 is_alive() const throw() = 0;
  virtual bool                 try_aliveness_restore() throw() = 0;

  virtual std::list<Interface *>  open_multiple_for_reading(const char *interface_type,
							      const char *id_pattern = "*") = 0;

  template <class InterfaceType>
    std::list<InterfaceType *>    open_multiple_for_reading(const char *id_pattern = "*");

  template <class InterfaceType>
    InterfaceType * open_for_reading(const char *identifier);

  template <class InterfaceType>
    InterfaceType * open_for_writing(const char *identifier);

  static const unsigned int BBIL_FLAG_DATA;
  static const unsigned int BBIL_FLAG_MESSAGES;
  static const unsigned int BBIL_FLAG_READER;
  static const unsigned int BBIL_FLAG_WRITER;
  static const unsigned int BBIL_FLAG_ALL;

  static const unsigned int BBIO_FLAG_CREATED;
  static const unsigned int BBIO_FLAG_DESTROYED;
  static const unsigned int BBIO_FLAG_ALL;

  virtual void register_listener(BlackBoardInterfaceListener *listener,
				 unsigned int flags) = 0;
  virtual void unregister_listener(BlackBoardInterfaceListener *listener) = 0;

  virtual void register_observer(BlackBoardInterfaceObserver *observer,
				 unsigned int flags) = 0;
  virtual void unregister_observer(BlackBoardInterfaceObserver *observer) = 0;

  char *  demangle_fawkes_interface_name(const char *type);
};


/** Get interface of given type.
 * This will open a new interface for reading just like the non-template version of
 * open_for_reading(). But with the template method you will get a correctly typed object
 * that you can use. An TypeMismatchException is thrown if the string representation
 * of the type and the actual class type of the interface do not match.
 * @param identifier identifier of the interface
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception TypeMismatchException thrown if type in interface_type and the actual class
 * type do not fit.
 */
template <class InterfaceType>
InterfaceType *
BlackBoard::open_for_reading(const char *identifier)
{
  char *type_name = demangle_fawkes_interface_name(typeid(InterfaceType).name());
  InterfaceType *interface = dynamic_cast<InterfaceType *>(open_for_reading(type_name, identifier));
  delete[] type_name;
  if ( interface == 0 ) {
    throw TypeMismatchException("Interface (R) types do not match");
  } else {
    return interface;
  }
}


/** Open all interfaces of given type for reading.
 * This will create interface instances for all currently registered interfaces of
 * the given type. The result can be casted to the appropriate type.
 * @param id_pattern pattern of interface IDs to open, supports wildcards similar
 * to filenames (*, ?, []), see "man fnmatch" for all supported.
 * @return list of new fully initialized interface instances of requested type. The
 * is allocated using new and you have to free it using delete after you are done
 * with it!
 */
template <class InterfaceType>
std::list<InterfaceType *>
BlackBoard::open_multiple_for_reading(const char *id_pattern)
{
  char *type_name = demangle_fawkes_interface_name(typeid(InterfaceType).name());
  std::list<Interface *> il = open_multiple_for_reading(type_name, id_pattern);
  delete[] type_name;
  std::list<InterfaceType *> rv;
  for (std::list<Interface *>::iterator i = il.begin(); i != il.end(); ++i) {
    InterfaceType *interface = dynamic_cast<InterfaceType *>(*i);
    if ( interface == 0 ) {
      // this really should never happen, but if it does we want to know,
      // because then we have a serious problem. We don't care about the
      // memleak here for that very reason.
      for (std::list<Interface *>::iterator j = il.begin(); j != il.end(); ++j) {
	close(*j);
      }
      throw TypeMismatchException("open_multiple_for_reading: "
				  "Interface (R) types do not match");
    } else {
      rv.push_back(interface);
    }
  }

  return rv;
}


/** Get writer interface of given type.
 * This will open a new interface for writing just like the non-template version of
 * open_for_writing(). But with the template method you will get a correctly typed object
 * that you can use. An TypeMismatchException is thrown if the string representation
 * of the type and the actual class type of the interface do not match.
 * @param identifier identifier of the interface
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception BlackBoardWriterActiveException thrown if there is already a writing
 * instance with the same type/id
 * @exception TypeMismatchException thrown if type in interface_type and the actual class
 * type do not fit.
 */
template <class InterfaceType>
InterfaceType *
BlackBoard::open_for_writing(const char *identifier)
{
  char *type_name = demangle_fawkes_interface_name(typeid(InterfaceType).name());
  InterfaceType *interface;
  try {
    interface = dynamic_cast<InterfaceType *>(open_for_writing(type_name, identifier));
  } catch (Exception &e) {
    // just caught to properly free memory
    delete[] type_name;
    throw;
  }
  delete[] type_name;
  if ( interface == 0 ) {
    throw TypeMismatchException("Interface (W) types do not match");
  } else {
    return interface;
  }
}

} // end namespace fawkes

#endif
