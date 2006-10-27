
/***************************************************************************
 *  message.cpp - BlackBoard message
 *
 *  Generated: Tue Oct 17 00:52:34 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <interfaces/message.h>

#include <core/threading/mutex.h>
#include <core/exceptions/software.h>

#include <string.h>
#include <stdlib.h>
#include <unistd.h>

/** @class Message interfaces/message.h
 * Base class for all messages passed through interfaces in Fawkes BlackBoard.
 * Do not use directly.
 */

/** @var Message::data_ptr
 * Pointer to memory that contains local data. This memory has to be allocated
 * by deriving classes with the approppriate size!
 */

/** @var Message::data_size
 * Size of memory needed to hold all data. This has to be set by deriving classes
 * to the appropriate value.
 */


/** Constructor */
Message::Message()
{
  message_id = 0;
  data_ptr = NULL;
  passing_interface_id = 0;
  originating_interface_id = 0;
}


/** Copy constructor.
 * @param mesg Message to copy.
 */
Message::Message(Message &mesg)
{
  message_id = 0;
  passing_interface_id = mesg.passing_interface_id;
  originating_interface_id = mesg.originating_interface_id;
  data_size = mesg.data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, mesg.data_ptr, data_size);
}


/** Copy constructor.
 * @param mesg Message to copy.
 */
Message::Message(Message *mesg)
{
  message_id = 0;
  passing_interface_id = mesg->passing_interface_id;
  originating_interface_id = mesg->originating_interface_id;
  data_size = mesg->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, mesg->data_ptr, data_size);
}


/** Virtual destructor.
 */
Message::~Message()
{
  if ( data_ptr != NULL ) {
    free(data_ptr);
    data_ptr = NULL;
  }
}


/** Get pointer to data.
 * Avoid usage.
 * @return pointer to internal data
 */
void *
Message::data()
{
  return data_ptr;
}


/** Get size of data.
 * @return size in bytes of data
 */
unsigned int
Message::datasize()
{
  return data_size;
}


/** Assign this message to given message.
 * Data is copied over from message if data sizes are the same.
 * @param m Message to copy
 * @return reference to current instance
 */
Message &
Message::operator=  (const Message & m)
{
  if ( data_size == m.data_size ) {
    memcpy(data_ptr, m.data_ptr, data_size);
  }

  return *this;
}
