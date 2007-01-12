
/***************************************************************************
 *  message_manager.cpp - BlackBoard message manager
 *
 *  Generated: Fri Oct 06 11:36:24 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

/** Constructor.
 * @param im interface manager to query for writer interface
 */
BlackBoardMessageManager::BlackBoardMessageManager(BlackBoardInterfaceManager *im)
{
  this->im = im;
}


/** Destructor */
BlackBoardMessageManager::~BlackBoardMessageManager()
{
}


void
BlackBoardMessageManager::transmit(Message *message)
{
  Interface *writer = im->getWriterForMemSerial(message->recipient_interface_mem_serial);
  writer->msgq_append(message);
}
