 
/***************************************************************************
 *  net_interface_listener.h - BlackBoard interface listener for net handler
 *
 *  Created: Tue Mar 04 17:50:54 2008
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

#ifndef __BLACKBOARD_NET_INTERFACE_LISTENER_H_
#define __BLACKBOARD_NET_INTERFACE_LISTENER_H_

#include <blackboard/interface_listener.h>

namespace fawkes {

class FawkesNetworkHub;
class BlackBoard;

class BlackBoardNetHandlerInterfaceListener
: public BlackBoardInterfaceListener
{
 public:
  BlackBoardNetHandlerInterfaceListener(BlackBoard *blackboard, Interface *interface,
					FawkesNetworkHub *hub, unsigned int clid);
  virtual ~BlackBoardNetHandlerInterfaceListener();

  virtual void bb_interface_data_changed(Interface *interface) throw();
  virtual bool bb_interface_message_received(Interface *interface, Message *message) throw();
  virtual void bb_interface_writer_added(Interface *interface,
					 unsigned int instance_serial) throw();
  virtual void bb_interface_writer_removed(Interface *interface,
					   unsigned int instance_serial) throw();
  virtual void bb_interface_reader_added(Interface *interface,
					 unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(Interface *interface,
					   unsigned int instance_serial) throw();

 private:
  void send_event_serial(Interface *interface, unsigned int msg_id, unsigned int event_serial);

  BlackBoard       *__blackboard;
  Interface        *__interface;
  FawkesNetworkHub *__fnh;

  unsigned int      __clid;
};

} // end namespace fawkes

#endif
