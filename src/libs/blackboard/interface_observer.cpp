 
/***************************************************************************
 *  interface_observer.cpp - BlackBoard interface observer
 *
 *  Created: Fri Jan 25 18:26:12 2008
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

#include <blackboard/interface_observer.h>
#include <interface/interface.h>
#include <cstdlib>
#include <cstring>

namespace fawkes {

/** @class BlackBoardInterfaceObserver <blackboard/interface_observer.h>
 * BlackBoard interface observer.
 * Derive this class if you want to be notified of specific BlackBoard 
 * events that are not tied to particular instances of interfaces like
 * create and destroy operations.
 *
 * The bb_interface_* methods are called during the appropriate operation. The
 * operation that you carry out in this event handler really has to be damn fast,
 * or the performance of the whole system will suffer severely. For this reason use
 * this notification facility only rarely and only register for the appropriate
 * events.
 *
 * This class provides the basic infrastructure that can be used to build
 * your own observer. During the life time of your observer you
 * first add all the interfaces to the appropriate structures that you want
 * to listen for and add the interface types where you want to be notified
 * of creation events.
 *
 * The interface created event is raised whenever an interface of a type that
 * you registered for is created. The destroyed event is raised if the an interface
 * is irrecoverable deleted from the BlackBoard. This happens when the last
 * reader or writer closes the interface. That means neither a writer nor any
 * reader has a particular interface still opened.
 *
 * Here is a simple life cycle of a BlackBoard interface observer:
 * First you add all the interface types that you want to observe with calls to
 * bbio_add_interface_create_type() and bbio_add_interface_destroy_type(). Then
 * you register the observer with InterfaceManager::register_observer(). From then
 * on you are notified of the events. Afterwards you unregister your observer
 * to no longer receive events.
 *
 * @author Tim Niemueller
 * @see BlackBoardInterfaceManager::register_observer()
 * @see BlackBoardInterfaceManager::unregister_observer()
 */

/** Empty constructor. */
BlackBoardInterfaceObserver::BlackBoardInterfaceObserver()
{
}

/** Destructor. */
BlackBoardInterfaceObserver::~BlackBoardInterfaceObserver()
{
  __bbio_observed_create.clear();
  __bbio_observed_destroy.clear();
}


/** BlackBoard interface created notification.
 * This is called whenever an interface is created for a type that you registered
 * for.
 * @param type type of the interface. If you want to store this make a copy as it
 * is not guaranteed that the supplied string exists for longer than the duration
 * of the method call
 * @param id ID of the newly created interface. If you want to store this make a
 * copy as it is not guaranteed that the supplied string exists for longer than
 * the duration of the method call
 */
void
BlackBoardInterfaceObserver::bb_interface_created(const char *type, const char *id) throw()
{
}


/** BlackBoard interface destroyed notification.
 * This is called whenever an interface is destroyed for a type that you registered
 * for.
 * @param type type of the interface. If you want to store this make a copy as it
 * is not guaranteed that the supplied string exists for longer than the duration
 * of the method call
 * @param id ID of the newly created interface. If you want to store this make a
 * copy as it is not guaranteed that the supplied string exists for longer than
 * the duration of the method call
 */
void
BlackBoardInterfaceObserver::bb_interface_destroyed(const char *type, const char *id) throw()
{
}


/** Add interface creation type to watch list.
 * With this you add an interface type to the watch list. For any type on this list
 * you will be notified if an interface is created.
 * @param type_pattern pattern of interface types to watch, supports wildcards
 * similar to filenames (*, ?, []), see "man fnmatch" for all supported.
 * @param id_pattern pattern of interface IDs to open, supports wildcards similar
 * to filenames (*, ?, []), see "man fnmatch" for all supported.
 */
void
BlackBoardInterfaceObserver::bbio_add_observed_create(const char *type_pattern,
						      const char *id_pattern) throw()
{
  __bbio_observed_create.lock();
  __bbio_observed_create[type_pattern].push_back(id_pattern);
  __bbio_observed_create[type_pattern].sort();
  __bbio_observed_create[type_pattern].unique();
  __bbio_observed_create.unlock();
}


/** Add interface destruction type to watch list.
 * With this you add an interface type to the watch list. For any type on this
 * list you will be notified if an interface is destroyed.
 * @param type_pattern pattern of interface types to watch, supports wildcards
 * similar to filenames (*, ?, []), see "man fnmatch" for all supported.
 * @param id_pattern pattern of interface IDs to open, supports wildcards similar
 * to filenames (*, ?, []), see "man fnmatch" for all supported.
 */
void
BlackBoardInterfaceObserver::bbio_add_observed_destroy(const char *type_pattern,
						       const char *id_pattern) throw()
{
  __bbio_observed_destroy.lock();
  __bbio_observed_destroy[type_pattern].push_back(id_pattern);
  __bbio_observed_destroy[type_pattern].sort();
  __bbio_observed_destroy[type_pattern].unique();
  __bbio_observed_destroy.unlock();
}


/** Get interface creation type watch list.
 * @return interface type watch list
 */
BlackBoardInterfaceObserver::ObservedInterfaceLockMap *
BlackBoardInterfaceObserver::bbio_get_observed_create() throw()
{
  return &__bbio_observed_create;
}


/** Get interface destriction type watch list.
 * @return interface type watch list
 */
BlackBoardInterfaceObserver::ObservedInterfaceLockMap *
BlackBoardInterfaceObserver::bbio_get_observed_destroy() throw()
{
  return &__bbio_observed_destroy;
}


} // end namespace fawkes
