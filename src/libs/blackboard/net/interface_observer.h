 
/***************************************************************************
 *  interface_observer.h - BlackBoard interface observer for net handler
 *
 *  Created: Wed Mar 02 17:02:35 2011
 *  Copyright  2007-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_NET_INTERFACE_OBSERVER_H_
#define __BLACKBOARD_NET_INTERFACE_OBSERVER_H_

#include <blackboard/interface_observer.h>

namespace fawkes {

class FawkesNetworkHub;
class BlackBoard;

class BlackBoardNetHandlerInterfaceObserver
: public BlackBoardInterfaceObserver
{
 public:
  BlackBoardNetHandlerInterfaceObserver(BlackBoard *blackboard, FawkesNetworkHub *hub);
  virtual ~BlackBoardNetHandlerInterfaceObserver();

  virtual void bb_interface_created(const char *type, const char *id) throw();
  virtual void bb_interface_destroyed(const char *type, const char *id) throw();

 private:
  void send_event(unsigned int msg_id, const char *type, const char *id);

 private:
  BlackBoard       *__blackboard;
  FawkesNetworkHub *__fnh;
};

} // end namespace fawkes

#endif
