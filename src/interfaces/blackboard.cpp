
/***************************************************************************
 *  blackboard.cpp - Fawkes BlackBoard Interface - BlackBoardInternalsInterface
 *
 *  Interface generated: Mon Oct 30 19:48:49 2006
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2006  Tim Niemueller
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
 *  along with this program; if not, write to the Free Software
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
 */

#include <interfaces/blackboard.h>

#include <string.h>
#include <stdlib.h>

/** @class BlackBoardInternalsInterface interfaces/blackboard.h
 * BlackBoardInternalsInterface Fawkes BlackBoard Interface.
 * This interface gives access to BlackBoard internal data. It is used by
      management code in the BlackBoard to communicate required data. It can also be
      used to allow for debugging from the outside.
 */



/** Constructor */
BlackBoardInternalsInterface::BlackBoardInternalsInterface() : Interface()
{
  data_size = sizeof(BlackBoardInternalsInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (BlackBoardInternalsInterface_data_t *)data_ptr;
}
/** Destructor */
BlackBoardInternalsInterface::~BlackBoardInternalsInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get InstanceSerial value.
 * A serial number incremented for each
    interface access instance of any type. This is not the internal memory serial which is
    incremented for every interface allocated in the shared memory! The instance serial
    allows messages to be linked with a specific sending interface instance, and not just
    with the interface block.
 * @return InstanceSerial value
 */
unsigned int
BlackBoardInternalsInterface::getInstanceSerial()
{
  return data->InstanceSerial;
}

/** Set InstanceSerial value.
 * A serial number incremented for each
    interface access instance of any type. This is not the internal memory serial which is
    incremented for every interface allocated in the shared memory! The instance serial
    allows messages to be linked with a specific sending interface instance, and not just
    with the interface block.
 * @param newInstanceSerial new InstanceSerial value
 */
void
BlackBoardInternalsInterface::setInstanceSerial(unsigned int newInstanceSerial)
{
  data->InstanceSerial = newInstanceSerial;
}

/* =========== messages =========== */
/** @class BlackBoardInternalsInterface::GetInstanceSerialMessage interfaces/blackboard.h
 * GetInstanceSerialMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
BlackBoardInternalsInterface::GetInstanceSerialMessage::GetInstanceSerialMessage() : Message()
{
  data_size = sizeof(GetInstanceSerialMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (GetInstanceSerialMessage_data_t *)data_ptr;
}
/** Destructor */
BlackBoardInternalsInterface::GetInstanceSerialMessage::~GetInstanceSerialMessage()
{
}
/* Methods */
/** @class BlackBoardInternalsInterface::GetMemSerialMessage interfaces/blackboard.h
 * GetMemSerialMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
BlackBoardInternalsInterface::GetMemSerialMessage::GetMemSerialMessage() : Message()
{
  data_size = sizeof(GetMemSerialMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (GetMemSerialMessage_data_t *)data_ptr;
}
/** Destructor */
BlackBoardInternalsInterface::GetMemSerialMessage::~GetMemSerialMessage()
{
}
/* Methods */
/** Check if message is valid an can be queued.
 * @param message Message to check
 */
bool
BlackBoardInternalsInterface::messageValid(const Message *message) const
{
  const GetInstanceSerialMessage *m0 = dynamic_cast<const GetInstanceSerialMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const GetMemSerialMessage *m1 = dynamic_cast<const GetMemSerialMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
Interface *
private_newBlackBoardInternalsInterface()
{
  return new BlackBoardInternalsInterface();
}
/// @endcond
/** Create instance of BlackBoardInternalsInterface.
 * @return instance of BlackBoardInternalsInterface
 */
extern "C"
Interface *
newBlackBoardInternalsInterface()
{
  return private_newBlackBoardInternalsInterface();
}

/** Destroy BlackBoardInternalsInterface instance.
 * @param interface BlackBoardInternalsInterface instance to destroy.
 */
extern "C"
void
deleteBlackBoardInternalsInterface(Interface *interface)
{
  delete interface;
}

