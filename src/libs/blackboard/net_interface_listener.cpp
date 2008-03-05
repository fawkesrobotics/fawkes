 
/***************************************************************************
 *  net_interface_listener.cpp - BlackBoard interface listener for net handler
 *
 *  Created: Tue Mar 04 17:53:32 2008
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

#include <blackboard/net_interface_listener.h>
#include <blackboard/net_messages.h>

#include <blackboard/blackboard.h>
#include <interface/interface.h>

#include <netcomm/fawkes/hub.h>
#include <netcomm/fawkes/message.h>
#include <netcomm/fawkes/component_ids.h>

/** @class BlackBoardNetHandlerInterfaceListener <blackboard/net_interface_listener.h>
 * Interface listener for network handler.
 * This class is used by the BlackBoardNetworkHandler to track interface changes and
 * send out notifications timely.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param blackboard local BlackBoard
 * @param interface interface to care about
 * @param hub Fawkes network hub to use to send messages
 * @param clid client ID of the client which opened this interface
 */
BlackBoardNetHandlerInterfaceListener::BlackBoardNetHandlerInterfaceListener(BlackBoard *blackboard,
									     Interface *interface,
									     FawkesNetworkHub *hub,
									     unsigned int clid)
{
  bbil_add_data_interface(interface);
  bbil_add_reader_interface(interface);
  bbil_add_writer_interface(interface);

  __blackboard = blackboard;
  __interface = interface;
  __fnh = hub;
  __clid = clid;

  __blackboard->register_listener(this, BlackBoard::BBIL_FLAG_ALL);
}


/** Destructor. */
BlackBoardNetHandlerInterfaceListener::~BlackBoardNetHandlerInterfaceListener()
{
  __blackboard->unregister_listener(this);
}


void
BlackBoardNetHandlerInterfaceListener::bb_interface_data_changed(Interface *interface) throw()
{
  // send out data changed notification
  interface->read();

  size_t payload_size = sizeof(bb_idata_msg_t) + interface->datasize();
  void *payload = malloc(payload_size);
  bb_idata_msg_t *dm = (bb_idata_msg_t *)payload;
  dm->serial = interface->serial();
  dm->data_size = interface->datasize();
  memcpy((char *)payload + sizeof(bb_idata_msg_t), interface->datachunk(),
	 interface->datasize());

  __fnh->send(__clid, FAWKES_CID_BLACKBOARD, MSG_BB_DATA_CHANGED, payload, payload_size);
}


void
BlackBoardNetHandlerInterfaceListener::send_serial(Interface *interface, unsigned int msg_id)
{
  bb_iserial_msg_t *sm = (bb_iserial_msg_t *)malloc(sizeof(bb_iserial_msg_t));
  sm->serial = interface->serial();

  __fnh->send(__clid, FAWKES_CID_BLACKBOARD, msg_id, sm, sizeof(bb_iserial_msg_t));  
}


void
BlackBoardNetHandlerInterfaceListener::bb_interface_writer_added(Interface *interface) throw()
{
  send_serial(interface, MSG_BB_WRITER_ADDED);
}


void
BlackBoardNetHandlerInterfaceListener::bb_interface_writer_removed(Interface *interface) throw()
{
  send_serial(interface, MSG_BB_WRITER_REMOVED);
}


void
BlackBoardNetHandlerInterfaceListener::bb_interface_reader_added(Interface *interface) throw()
{
  send_serial(interface, MSG_BB_READER_ADDED);
}


void
BlackBoardNetHandlerInterfaceListener::bb_interface_reader_removed(Interface *interface) throw()
{
  send_serial(interface, MSG_BB_READER_REMOVED);
}
