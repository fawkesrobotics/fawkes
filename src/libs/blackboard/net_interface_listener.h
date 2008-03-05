 
/***************************************************************************
 *  net_interface_listener.h - BlackBoard interface listener for net handler
 *
 *  Created: Tue Mar 04 17:50:54 2008
 *  Copyright  2007-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_NET_INTERFACE_LISTENER_H_
#define __BLACKBOARD_NET_INTERFACE_LISTENER_H_

#include <blackboard/interface_listener.h>

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
  virtual void bb_interface_writer_added(Interface *interface) throw();
  virtual void bb_interface_writer_removed(Interface *interface) throw();
  virtual void bb_interface_reader_added(Interface *interface) throw();
  virtual void bb_interface_reader_removed(Interface *interface) throw();

 private:
  void send_serial(Interface *interface, unsigned int msg_id);

  BlackBoard       *__blackboard;
  Interface        *__interface;
  FawkesNetworkHub *__fnh;

  unsigned int      __clid;
};


#endif
