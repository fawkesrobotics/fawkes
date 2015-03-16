 
/***************************************************************************
 *  interface_list_maintainer.cpp - BlackBoard interface list maintainer
 *
 *  Created: Mon Mar 16 13:34:00 2015
 *  Copyright  2007-2014  Tim Niemueller [www.niemueller.de]
 *             2015       Tobias Neumann
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

#include "interface_list_maintainer.h"

#include <core/threading/mutex_locker.h>
#include <algorithm>
#include <string.h>

using namespace fawkes;

/** @class BlackBoardInterfaceListMaintainer "interface_list_maintainer.h"
 * opens and maintains multiple interfaces defined by a pattern
 * @author Tobias Neumann
 */

/** Constructor
 * @param n       name of plugin to use this
 * @param bb      pointer to the BlackBoard given by abstract
 * @param l       pointer to the Logger given by abstract
 * @param type    the interface type
 * @param pattern the pattern for interfaces to open
 *
 */
BlackBoardInterfaceListMaintainer::BlackBoardInterfaceListMaintainer(const char* n, BlackBoard* bb, Logger* l, const char *type, const char *pattern)
: BlackBoardInterfaceListener(n)
{
  blackboard_  = bb;
  logger_      = l;
  name_        = strdup(n);

  bbio_add_observed_create(type, pattern);
  blackboard_->register_observer(this);

  MutexLocker lock(ifs_.mutex());

  // for all interfaces of my pattern
  std::list<fawkes::Interface *> ifs_tmp = blackboard_->open_multiple_for_reading(type, pattern);
  std::list<fawkes::Interface *>::iterator pif_tmp;
  for ( pif_tmp = ifs_tmp.begin(); pif_tmp != ifs_tmp.end(); ++pif_tmp ) {
    // check if this is allready opened by me
    std::string id_list_tmp( (*pif_tmp)->id() );
    bool is_in_list = false;
    fawkes::LockList<fawkes::Interface *>::iterator pif_class;
    for ( pif_class = ifs_.begin(); pif_class != ifs_.end(); ++pif_class ) {
      std::string id_list_class( (*pif_class)->id() );

      if ( id_list_tmp.compare( id_list_class ) == 0 ) {
        blackboard_->close( *pif_tmp );
        is_in_list = true;
      }
    }

    if ( ! is_in_list ) {
      ifs_.push_back( (*pif_tmp) );
    }

    bbil_add_reader_interface((*pif_tmp));
    bbil_add_writer_interface((*pif_tmp));
  }
  blackboard_->register_listener(this);

  lock.unlock();
}

/** Destructor. */
BlackBoardInterfaceListMaintainer::~BlackBoardInterfaceListMaintainer()
{
  delete(name_);

  MutexLocker lock( ifs_.mutex() );
  fawkes::LockList<fawkes::Interface *>::iterator pif;
  for ( pif = ifs_.begin(); pif != ifs_.end(); ++pif ) {
    bbil_remove_writer_interface( *pif );
    bbil_remove_reader_interface( *pif );
    blackboard_->update_listener(this);
    blackboard_->close( *pif );
  }
}

/**
 * Callback if interface defined by the pattern is created.
 * Now we try to open it.
 * @param type  the type of the created interface
 * @param id    the name of the interface
 */
void
BlackBoardInterfaceListMaintainer::bb_interface_created(const char *type, const char *id) throw()
{
  Interface *pif;
  try {
    pif = blackboard_->open_for_reading(type, id);
  } catch (Exception &e) {
    // ignored
    logger_->log_warn(name_, "Failed to open %s:%s: %s", type, id, e.what_no_backtrace());
    return;
  }

  try {
    bbil_add_reader_interface(pif);
    bbil_add_writer_interface(pif);
    blackboard_->update_listener(this);
  } catch (Exception &e) {
    logger_->log_warn(name_, "Failed to register for %s:%s: %s",
                     type, id, e.what());
    try {
      bbil_remove_reader_interface(pif);
      bbil_remove_writer_interface(pif);
      blackboard_->update_listener(this);
      blackboard_->close(pif);
    } catch (Exception &e) {
      logger_->log_error(name_, "Failed to deregister %s:%s during error recovery: %s",
                        type, id, e.what());
    }
    return;
  }

  ifs_.push_back_locked(pif);
}

/**
 * Callback if writer is removed from an interface. Now we check if we can close this interface.
 * @param interface       the interface that raised the callback
 * @param instance_serial defiend by the callback, not used here
 */
void
BlackBoardInterfaceListMaintainer::bb_interface_writer_removed(fawkes::Interface *interface,
                unsigned int instance_serial) throw()
{
  conditional_close(interface);
}

/**
 * Callback if a reader is removed from an interface. Now we check if we can close this interface.
 * @param interface       the interface that raised the callback
 * @param instance_serial defiend by the callback, not used here
 */
void
BlackBoardInterfaceListMaintainer::bb_interface_reader_removed(fawkes::Interface *interface,
                unsigned int instance_serial) throw()
{
  conditional_close(interface);
}

/**
 * Checks if the given interface can be closed and does so if possible.
 * The check is, no writer and just one reader (us)
 * @param pif interface to close
 */
void
BlackBoardInterfaceListMaintainer::conditional_close(Interface *pif) throw()
{
  bool close = false;
  MutexLocker lock(ifs_.mutex());

  LockList<Interface *>::iterator c =
    std::find(ifs_.begin(), ifs_.end(), pif);

  if (c != ifs_.end() &&
      (! pif->has_writer() && (pif->num_readers() == 1)))
  {
    // It's only us
    logger_->log_info(name_, "Last on %s, closing", pif->uid());
    close = true;
    ifs_.erase(c);
  }

  lock.unlock();

  if (close) {
    std::string uid = pif->uid();
    try {
      bbil_remove_reader_interface(pif);
      bbil_remove_writer_interface(pif);
      blackboard_->update_listener(this);
      blackboard_->close(pif);
    } catch (Exception &e) {
      logger_->log_error(name_, "Failed to unregister or close %s: %s",
                        uid.c_str(), e.what());
    }
  }
}

/** unlocks the mutex in this class
 *
 * this method needs to be called after lock_and_get_list()
 * the list returned by lock_and_get_list() is invalid and shouldn't be used after this method is called
 */
void
BlackBoardInterfaceListMaintainer::unlock_list()
{
  ifs_.unlock();
}
