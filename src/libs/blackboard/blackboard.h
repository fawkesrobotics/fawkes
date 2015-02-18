
/***************************************************************************
 *  blackboard.h - BlackBoard Interface
 *
 *  Created: Sat Sep 16 17:09:15 2006 (on train to Cologne)
 *  Copyright  2006-2015  Tim Niemueller [www.niemueller.de]
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
#include <interface/interface.h>

#include <list>
#include <string>
#include <typeinfo>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class BlackBoardInterfaceManager;
class BlackBoardMemoryManager;
class BlackBoardMessageManager;
class BlackBoardNetworkHandler;
class BlackBoardNotifier;
class InterfaceInfoList;
class BlackBoardInterfaceListener;
class BlackBoardInterfaceObserver;
class FawkesNetworkHub;

class BlackBoard
{
 public:
  virtual ~BlackBoard();

  virtual Interface *  open_for_reading(const char *interface_type,
                                        const char *identifier,
					const char *owner = NULL) = 0;
  virtual Interface *  open_for_writing(const char *interface_type,
                                        const char *identifier,
					const char *owner = NULL) = 0;
  virtual void         close(Interface *interface) = 0;

  virtual Interface *  open_for_reading_f(const char *interface_type,
					  const char *identifier, ...);
  virtual Interface *  open_for_writing_f(const char *interface_type,
					  const char *identifier, ...);


  virtual InterfaceInfoList *  list_all() = 0;
  virtual InterfaceInfoList *  list(const char *type_pattern,
				    const char *id_pattern) = 0;
  virtual bool                 is_alive() const throw() = 0;
  virtual bool                 try_aliveness_restore() throw() = 0;

  virtual std::list<Interface *>
    open_multiple_for_reading(const char *type_pattern,
                              const char *id_pattern = "*",
			      const char *owner = NULL) = 0;

  template <class InterfaceType>
  std::list<InterfaceType *>
    open_multiple_for_reading(const char *id_pattern = "*",
			      const char *owner = NULL);

  template <class InterfaceType>
    InterfaceType * open_for_reading(const char *identifier,
				     const char *owner = NULL);

  template <class InterfaceType>
    InterfaceType * open_for_writing(const char *identifier,
				     const char *owner = NULL);

  template <class InterfaceType>
    InterfaceType * open_for_reading_f(const char *identifier, ...);

  template <class InterfaceType>
    InterfaceType * open_for_writing_f(const char *identifier, ...);

  /** Flags to constrain listener registration/updates. */
  typedef enum {
    BBIL_FLAG_DATA = 1,		///< consider data events
    BBIL_FLAG_MESSAGES = 2,	///< consider message received events
    BBIL_FLAG_READER = 4,	///< consider reader events
    BBIL_FLAG_WRITER = 8,	///< consider writer events
    BBIL_FLAG_ALL = 15,		///< consider all events
  } ListenerRegisterFlag;

  virtual void register_listener(BlackBoardInterfaceListener *listener,
                                 ListenerRegisterFlag flag = BBIL_FLAG_ALL);
  virtual void update_listener(BlackBoardInterfaceListener *listener,
                                 ListenerRegisterFlag flag = BBIL_FLAG_ALL);
  virtual void unregister_listener(BlackBoardInterfaceListener *listener);

  virtual void register_observer(BlackBoardInterfaceObserver *observer);
  virtual void unregister_observer(BlackBoardInterfaceObserver *observer);

  std::string  demangle_fawkes_interface_name(const char *type);
  std::string  format_identifier(const char *identifier_format, va_list arg);

 protected:
  BlackBoard(bool create_notifier = true);

 protected:
  BlackBoardNotifier *__notifier;	///< Notifier for BB events.
};


/** Get interface of given type.
 * This will open a new interface for reading just like the
 * non-template version of open_for_reading(). But with the template
 * method you will get a correctly typed object that you can use. An
 * TypeMismatchException is thrown if the string representation of the
 * type and the actual class type of the interface do not match.
 * @param identifier identifier of the interface
 * @param owner name of entity which opened this interface. If using the BlackBoardAspect
 * to access the blackboard leave this untouched unless you have a good reason.
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception TypeMismatchException thrown if type in interface_type
 * and the actual class type do not fit.
 */
template <class InterfaceType>
InterfaceType *
BlackBoard::open_for_reading(const char *identifier, const char *owner)
{
  std::string type_name =
    demangle_fawkes_interface_name(typeid(InterfaceType).name());
  Interface *interface = open_for_reading(type_name.c_str(), identifier, owner);
  return static_cast<InterfaceType *>(interface);
}


/** Get interface of given type with identifier format string.
 * This will open a new interface for reading just like the
 * non-template version of open_for_reading(). But with the template
 * method you will get a correctly typed object that you can use. An
 * TypeMismatchException is thrown if the string representation of the
 * type and the actual class type of the interface do not match.
 * @param identifier identifier of the interface
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception TypeMismatchException thrown if type in interface_type
 * and the actual class type do not fit.
 */
template <class InterfaceType>
InterfaceType *
BlackBoard::open_for_reading_f(const char *identifier, ...)
{
  va_list arg;
  va_start(arg, identifier);
  std::string type_name =
    demangle_fawkes_interface_name(typeid(InterfaceType).name());
  std::string identifier_s = format_identifier(identifier, arg);
  va_end(arg);
  Interface *interface = open_for_reading(type_name.c_str(), identifier_s.c_str());
  return static_cast<InterfaceType *>(interface);
}


/** Open all interfaces of given type for reading.
 * This will create interface instances for all currently registered interfaces of
 * the given type. The result can be casted to the appropriate type.
 * @param id_pattern pattern of interface IDs to open, supports wildcards similar
 * to filenames (*, ?, []), see "man fnmatch" for all supported.
 * @param owner name of entity which opened this interface. If using the BlackBoardAspect
 * to access the blackboard leave this untouched unless you have a good reason.
 * @return list of new fully initialized interface instances of requested type. The
 * is allocated using new and you have to free it using delete after you are done
 * with it!
 */
template <class InterfaceType>
std::list<InterfaceType *>
BlackBoard::open_multiple_for_reading(const char *id_pattern, const char *owner)
{
  std::string type_name =
    demangle_fawkes_interface_name(typeid(InterfaceType).name());
  std::list<Interface *> il =
    open_multiple_for_reading(type_name.c_str(), id_pattern, owner);
  std::list<InterfaceType *> rv;
  for (std::list<Interface *>::iterator i = il.begin(); i != il.end(); ++i) {
    rv.push_back(static_cast<InterfaceType *>(*i));
  }

  return rv;
}


/** Get writer interface of given type.
 * This will open a new interface for writing just like the
 * non-template version of open_for_writing(). But with the template
 * method you will get a correctly typed object that you can use. An
 * TypeMismatchException is thrown if the string representation of the
 * type and the actual class type of the interface do not match.
 * @param identifier identifier of the interface
 * @param owner name of entity which opened this interface. If using the BlackBoardAspect
 * to access the blackboard leave this untouched unless you have a good reason.
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception BlackBoardWriterActiveException thrown if there is already a writing
 * instance with the same type/id
 * @exception TypeMismatchException thrown if type in interface_type
 * and the actual class type do not fit.
 */
template <class InterfaceType>
InterfaceType *
BlackBoard::open_for_writing(const char *identifier, const char *owner)
{
  std::string type_name =
    demangle_fawkes_interface_name(typeid(InterfaceType).name());
  Interface *interface = open_for_writing(type_name.c_str(), identifier, owner);
  return static_cast<InterfaceType *>(interface);;
}


/** Get writer interface of given type with identifier format string.
 * This will open a new interface for writing just like the
 * non-template version of open_for_writing(). But with the template
 * method you will get a correctly typed object that you can use. An
 * TypeMismatchException is thrown if the string representation of the
 * type and the actual class type of the interface do not match.
 * @param identifier identifier of the interface
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception BlackBoardWriterActiveException thrown if there is already a writing
 * instance with the same type/id
 * @exception TypeMismatchException thrown if type in interface_type
 * and the actual class type do not fit.
 */
template <class InterfaceType>
InterfaceType *
BlackBoard::open_for_writing_f(const char *identifier, ...)
{
  va_list arg;
  va_start(arg, identifier);
  std::string type_name =
    demangle_fawkes_interface_name(typeid(InterfaceType).name());
  std::string identifier_s = format_identifier(identifier, arg);
  va_end(arg);
  Interface *interface = open_for_writing(type_name.c_str(), identifier_s.c_str());
  return static_cast<InterfaceType *>(interface);;
}


/** Concatenation of register flags.
 * @param a flags to concatenate
 * @param b other flags to concatenate
 * @return concatenated flags
 */
inline BlackBoard::ListenerRegisterFlag
operator|(const BlackBoard::ListenerRegisterFlag &a,
          const BlackBoard::ListenerRegisterFlag &b)
{
  return (BlackBoard::ListenerRegisterFlag)((int)a | (int)b);
}


/** Testing of register flags.
 * @param a flags to test
 * @param b flags to test for
 * @return resulting flags
 */
inline BlackBoard::ListenerRegisterFlag
operator&(const BlackBoard::ListenerRegisterFlag &a,
          const BlackBoard::ListenerRegisterFlag &b)
{
  return (BlackBoard::ListenerRegisterFlag)((int)a & (int)b);
}


} // end namespace fawkes

#endif
