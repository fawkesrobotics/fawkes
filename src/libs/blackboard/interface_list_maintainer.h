 
/***************************************************************************
 *  interface_list_maintainer.h - BlackBoard interface list maintainer
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

#ifndef __BLACKBOARD_INTERFACE_LIST_MAINTAINER_H_
#define __BLACKBOARD_INTERFACE_LIST_MAINTAINER_H_

#include <logging/logger.h>
#include <blackboard/blackboard.h>

#include <core/utils/lock_list.h>
#include <interface/interface.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>

#include <list>
#include <string>

namespace fawkes {

/** @class BlackBoardInterfaceListMaintainer "interface_list_maintainer.h"
 * opens and maintains multiple interfaces defined by a pattern
 * @author Tobias Neumann
 */
class BlackBoardInterfaceListMaintainer
:
  public fawkes::BlackBoardInterfaceObserver,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  BlackBoardInterfaceListMaintainer(const char* n, BlackBoard* bb, Logger* l, const char *type, const char *pattern);
  virtual ~BlackBoardInterfaceListMaintainer();

  template <class InterfaceType>
  std::list<InterfaceType *> lock_and_get_list();

  void unlock_list();

 private:
  // for BlackBoardInterfaceObserver
  virtual void bb_interface_created(const char *type, const char *id) throw();

  // for BlackBoardInterfaceListener
  virtual void bb_interface_writer_removed(fawkes::Interface *interface,
                                          unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(fawkes::Interface *interface,
                                          unsigned int instance_serial) throw();

  void conditional_close(fawkes::Interface *interface) throw();

 private:
  BlackBoard                            *blackboard_;
  Logger                                *logger_;
  const char                            *name_;
  fawkes::LockList<fawkes::Interface *> ifs_;
};

/** Locks the mutex in this class and returns a list of all interfaces defined by the pattern
 *
 * after the list is used unlock_list() needs to be called to unlock the mutex in this class
 *
 * @return list of interfaces defined by the pattern
 */
template <class InterfaceType>
std::list<InterfaceType *>
BlackBoardInterfaceListMaintainer::lock_and_get_list()
{
  ifs_.lock();
  std::list<InterfaceType *> ifs_cpy;
  for ( fawkes::LockList<fawkes::Interface *>::iterator pif = ifs_.begin();
        pif != ifs_.end();
        ++pif ) {
    (*pif)->read();
    ifs_cpy.push_back( dynamic_cast<InterfaceType*> (*pif) );
  }
  return ifs_cpy;
}

} // end namespace fawkes

#endif
