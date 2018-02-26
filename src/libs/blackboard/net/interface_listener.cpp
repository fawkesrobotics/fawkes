 
/***************************************************************************
 *  net_interface_listener.cpp - BlackBoard interface listener for net handler
 *
 *  Created: Tue Mar 04 17:53:32 2008
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

#include <blackboard/net/interface_listener.h>
#include <blackboard/net/messages.h>

#include <blackboard/blackboard.h>
#include <interface/interface.h>

#include <netcomm/fawkes/hub.h>
#include <netcomm/fawkes/message.h>
#include <netcomm/fawkes/component_ids.h>
#include <logging/liblogger.h>

#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <arpa/inet.h>

namespace fawkes {

/** @class BlackBoardNetHandlerInterfaceListener <blackboard/net/interface_listener.h>
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
  : BlackBoardInterfaceListener("NetIL/%s", interface->uid())
{
  bbil_add_data_interface(interface);
  bbil_add_reader_interface(interface);
  bbil_add_writer_interface(interface);
  if ( interface->is_writer() ) {
    bbil_add_message_interface(interface);
  }

  __blackboard = blackboard;
  __interface = interface;
  __fnh = hub;
  __clid = clid;

  __blackboard->register_listener(this);
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
  void *payload       = malloc(payload_size);
  bb_idata_msg_t *dm  = (bb_idata_msg_t *)payload;
  dm->serial          = htonl(interface->serial());
  dm->data_size       = htonl(interface->datasize());
  memcpy((char *)payload + sizeof(bb_idata_msg_t), interface->datachunk(),
	 interface->datasize());

  try {
    __fnh->send(__clid, FAWKES_CID_BLACKBOARD, MSG_BB_DATA_CHANGED, payload, payload_size);
  } catch (Exception &e) {
    LibLogger::log_warn(bbil_name(), "Failed to send BlackBoard data, exception follows");
    LibLogger::log_warn(bbil_name(), e);
  }
}


bool
BlackBoardNetHandlerInterfaceListener::bb_interface_message_received(Interface *interface,
								     Message *message) throw()
{
  // send out interface message
  size_t payload_size = sizeof(bb_imessage_msg_t) + message->datasize();
  void *payload = calloc(1, payload_size);
  bb_imessage_msg_t *dm = (bb_imessage_msg_t *)payload;
  dm->serial = htonl(interface->serial());
  strncpy(dm->msg_type, message->type(), __INTERFACE_MESSAGE_TYPE_SIZE-1);
  dm->data_size = htonl(message->datasize());
  dm->msgid = htonl(message->id());
  dm->hops  = htonl(message->hops());
  memcpy((char *)payload + sizeof(bb_imessage_msg_t), message->datachunk(),
	 message->datasize());

  try {
    __fnh->send(__clid, FAWKES_CID_BLACKBOARD, MSG_BB_INTERFACE_MESSAGE, payload, payload_size);
  } catch (Exception &e) {
    LibLogger::log_warn(bbil_name(), "Failed to send BlackBoard message, exception follows");
    LibLogger::log_warn(bbil_name(), e);
  }

  // do not enqueue, we are fine with just sending
  return false;
}


void
BlackBoardNetHandlerInterfaceListener::send_event_serial(Interface *interface,
							 unsigned int msg_id,
							 unsigned int event_serial)
{
  bb_ieventserial_msg_t *esm = (bb_ieventserial_msg_t *)malloc(sizeof(bb_ieventserial_msg_t));
  esm->serial       = htonl(interface->serial());
  esm->event_serial = htonl(event_serial);

  try {
    __fnh->send(__clid, FAWKES_CID_BLACKBOARD, msg_id, esm, sizeof(bb_ieventserial_msg_t));  
  } catch (Exception &e) {
    LibLogger::log_warn(bbil_name(), "Failed to send BlackBoard event serial, exception follows");
    LibLogger::log_warn(bbil_name(), e);
  }
}


void
BlackBoardNetHandlerInterfaceListener::bb_interface_writer_added(Interface *interface,
								 unsigned int instance_serial) throw()
{
  send_event_serial(interface, MSG_BB_WRITER_ADDED, instance_serial);
}


void
BlackBoardNetHandlerInterfaceListener::bb_interface_writer_removed(Interface *interface,
								   unsigned int instance_serial) throw()
{
  send_event_serial(interface, MSG_BB_WRITER_REMOVED, instance_serial);
}


void
BlackBoardNetHandlerInterfaceListener::bb_interface_reader_added(Interface *interface,
								 unsigned int instance_serial) throw()
{
  send_event_serial(interface, MSG_BB_READER_ADDED, instance_serial);
}


void
BlackBoardNetHandlerInterfaceListener::bb_interface_reader_removed(Interface *interface,
								   unsigned int instance_serial) throw()
{
  send_event_serial(interface, MSG_BB_READER_REMOVED, instance_serial);
}

} // end namespace fawkes
