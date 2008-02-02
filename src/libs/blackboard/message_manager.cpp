
/***************************************************************************
 *  message_manager.cpp - BlackBoard message manager
 *
 *  Created: Fri Oct 06 11:36:24 2006
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/message_manager.h>
#include <blackboard/interface_manager.h>
#include <blackboard/exceptions.h>

#include <interface/message.h>
#include <interface/interface.h>

#include <core/exceptions/software.h>
#include <utils/logging/liblogger.h>


/** @class BlackBoardMessageManager <blackboard/message_manager.h>
 * BlackBoard message manager.
 * Transmits messages from reading interface instances to the writer instance
 * if the interface, if there is any.
 * @author Tim Niemueller
 */

/** Constructor. */
BlackBoardMessageManager::BlackBoardMessageManager()
{
  __im = NULL;
}


/** Destructor */
BlackBoardMessageManager::~BlackBoardMessageManager()
{
}


void
BlackBoardMessageManager::transmit(Message *message)
{
  if ( __im == NULL ) {
    throw NullPointerException("InterfaceManager has not been set for MessageManager");
  }
  try {
    Interface *writer = __im->writer_for_mem_serial(message->recipient_interface_mem_serial);
    writer->msgq_append(message);
  } catch (BlackBoardNoWritingInstanceException &e) {
    Interface *iface = message->interface();
    LibLogger::log_warn("BlackBoardMessageManager", "Cannot transmit message from sender %s "
			                            "via interface %s (type %s), no writing "
			                            "instance exists!",
			message->sender(), (iface != NULL) ? iface->id() : "Unknown",
			(iface != NULL) ? iface->type() : "unknown");
  }

}


/** Set interface manager.
 * @param im interface manager
 */
void
BlackBoardMessageManager::set_interface_manager(BlackBoardInterfaceManager *im)
{
  __im = im;
}
