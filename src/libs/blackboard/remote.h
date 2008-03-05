
/***************************************************************************
 *  remote.h - Remote BlackBoard using the Fawkes network protocol
 *
 *  Created: Mon Mar 03 10:52:28 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_REMOTE_H_
#define __BLACKBOARD_REMOTE_H_

#include <blackboard/blackboard.h>
#include <netcomm/fawkes/client_handler.h>
#include <core/utils/lock_map.h>

class FawkesNetworkClient;
class FawkesNetworkMessage;
class Mutex;

class BlackBoardInstanceFactory;
class BlackBoardNotifier;
class BlackBoardInterfaceProxy;

class RemoteBlackBoard : public FawkesNetworkClientHandler
{
 public:
  RemoteBlackBoard(FawkesNetworkClient *client);
  RemoteBlackBoard(const char *hostname, unsigned short int port);
  virtual ~RemoteBlackBoard();

  Interface *  open_for_reading(const char *interface_type, const char *identifier);
  Interface *  open_for_writing(const char *interface_type, const char *identifier);
  void         close(Interface *interface);

  InterfaceInfoList *  list_all();

  std::list<Interface *> *  open_all_of_type_for_reading(const char *interface_type,
							 const char *id_prefix = NULL);

  template <class InterfaceType>
    std::list<InterfaceType *> *  open_all_of_type_for_reading(const char *id_prefix = NULL);

  template <class InterfaceType>
    InterfaceType * open_for_reading(const char *identifier);

  template <class InterfaceType>
    InterfaceType * open_for_writing(const char *identifier);


  void register_listener(BlackBoardInterfaceListener *listener, unsigned int flags);
  void unregister_listener(BlackBoardInterfaceListener *listener);

  void register_observer(BlackBoardInterfaceObserver *observer, unsigned int flags);
  void unregister_observer(BlackBoardInterfaceObserver *observer);


  /* for FawkesNetworkClientHandler */
  virtual void          deregistered() throw();
  virtual void          inbound_received(FawkesNetworkMessage *msg) throw();
  virtual void          connection_died() throw();
  virtual void          connection_established() throw();


  /* extensions for RemoteBlackBoard */

 private: /* methods */
  Interface * open_interface(const char *type, const char *identifier, bool writer);


 private: /* members */
  Mutex *__mutex;
  FawkesNetworkClient  *__fnc;
  bool                  __fnc_owner;
  FawkesNetworkMessage *__m;
  BlackBoardNotifier   *__notifier;
  BlackBoardInstanceFactory *__instance_factory;
  LockMap<unsigned int, BlackBoardInterfaceProxy *> __proxies;
  LockMap<unsigned int, BlackBoardInterfaceProxy *>::iterator __pit;
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
RemoteBlackBoard::open_for_reading(const char *identifier)
{
  char *type_name = BlackBoard::strip_class_type(typeid(InterfaceType).name());
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
 * @param id_prefix if set only interfaces whose ids have this prefix are returned
 * @return list of new fully initialized interface instances of requested type. The
 * is allocated using new and you have to free it using delete after you are done
 * with it!
 */
template <class InterfaceType>
std::list<InterfaceType *> *
RemoteBlackBoard::open_all_of_type_for_reading(const char *id_prefix)
{
  char *type_name = BlackBoard::strip_class_type(typeid(InterfaceType).name());
  std::list<Interface *> *il = open_all_of_type_for_reading(type_name, id_prefix);
  delete[] type_name;
  std::list<InterfaceType *> *rv = new std::list<InterfaceType *>();
  for (std::list<Interface *>::iterator i = il->begin(); i != il->end(); ++i) {
    InterfaceType *interface = dynamic_cast<InterfaceType *>(*i);
    if ( interface == 0 ) {
      // this really should never happen, but if it does we want to know,
      // because then we have a serious problem. We don't care about the
      // memleak here for that very reason.
      throw TypeMismatchException("open_all_off_type_for_reading: "
				  "Interface (R) types do not match");
    } else {
      rv->push_back(interface);
    }
  }

  delete il;

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
RemoteBlackBoard::open_for_writing(const char *identifier)
{
  char *type_name = BlackBoard::strip_class_type(typeid(InterfaceType).name());
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

#endif
