 
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
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __BLACKBOARD_INTERFACE_OBSERVER_H_
#define __BLACKBOARD_INTERFACE_OBSERVER_H_

#include <map>
#include <list>
#include <string>

namespace fawkes {

class BlackBoardInterfaceObserver
{

 friend class BlackBoardNotifier;

 public:
  BlackBoardInterfaceObserver();
  virtual ~BlackBoardInterfaceObserver();

  virtual void bb_interface_created(const char *type, const char *id) throw();
  virtual void bb_interface_destroyed(const char *type, const char *id) throw();

 protected:
  void bbio_add_observed_create(const char *type, const char *id_pattern = "*") throw();
  void bbio_add_observed_destroy(const char *type, const char *id_pattern = "*") throw();

  /** Type for lockable interface type hash sets. */
  typedef  std::map<std::string, std::list<std::string> >  ObservedInterfaceMap;

  /** Type for iterator of lockable interface type hash sets. */
  typedef  ObservedInterfaceMap::iterator   ObservedInterfaceMapIterator;

  ObservedInterfaceMap *  bbio_get_observed_create() throw();
  ObservedInterfaceMap *  bbio_get_observed_destroy() throw();

 private:
  ObservedInterfaceMap         __bbio_observed_create;
  ObservedInterfaceMap         __bbio_observed_destroy;
  ObservedInterfaceMapIterator __bbio_iti;
};

} // end namespace fawkes

#endif
