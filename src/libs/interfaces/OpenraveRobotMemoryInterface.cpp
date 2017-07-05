
/***************************************************************************
 *  OpenraveRobotMemoryInterface.cpp - Fawkes BlackBoard Interface - OpenraveRobotMemoryInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  Frederik Zwilling
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

#include <interfaces/OpenraveRobotMemoryInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class OpenraveRobotMemoryInterface <interfaces/OpenraveRobotMemoryInterface.h>
 * OpenraveRobotMemoryInterface Fawkes BlackBoard Interface.
 * 
      Interface to instruct the OpenraveRobotMemory Plugin
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
OpenraveRobotMemoryInterface::OpenraveRobotMemoryInterface() : Interface()
{
  data_size = sizeof(OpenraveRobotMemoryInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (OpenraveRobotMemoryInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "dummy", 1, &data->dummy);
  add_messageinfo("ConstructSceneMessage");
  unsigned char tmp_hash[] = {0x49, 0x41, 0x1e, 0x3, 0xf2, 0xeb, 0x23, 0xb8, 0x2a, 0x6e, 0x90, 0xc2, 0x3e, 0xe9, 0xa4, 0x24};
  set_hash(tmp_hash);
}

/** Destructor */
OpenraveRobotMemoryInterface::~OpenraveRobotMemoryInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get dummy value.
 * 
      Dummy field
    
 * @return dummy value
 */
uint32_t
OpenraveRobotMemoryInterface::dummy() const
{
  return data->dummy;
}

/** Get maximum length of dummy value.
 * @return length of dummy value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenraveRobotMemoryInterface::maxlenof_dummy() const
{
  return 1;
}

/** Set dummy value.
 * 
      Dummy field
    
 * @param new_dummy new dummy value
 */
void
OpenraveRobotMemoryInterface::set_dummy(const uint32_t new_dummy)
{
  data->dummy = new_dummy;
  data_changed = true;
}

/* =========== message create =========== */
Message *
OpenraveRobotMemoryInterface::create_message(const char *type) const
{
  if ( strncmp("ConstructSceneMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ConstructSceneMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
OpenraveRobotMemoryInterface::copy_values(const Interface *other)
{
  const OpenraveRobotMemoryInterface *oi = dynamic_cast<const OpenraveRobotMemoryInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(OpenraveRobotMemoryInterface_data_t));
}

const char *
OpenraveRobotMemoryInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class OpenraveRobotMemoryInterface::ConstructSceneMessage <interfaces/OpenraveRobotMemoryInterface.h>
 * ConstructSceneMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
OpenraveRobotMemoryInterface::ConstructSceneMessage::ConstructSceneMessage() : Message("ConstructSceneMessage")
{
  data_size = sizeof(ConstructSceneMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ConstructSceneMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
OpenraveRobotMemoryInterface::ConstructSceneMessage::~ConstructSceneMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenraveRobotMemoryInterface::ConstructSceneMessage::ConstructSceneMessage(const ConstructSceneMessage *m) : Message("ConstructSceneMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ConstructSceneMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenraveRobotMemoryInterface::ConstructSceneMessage::clone() const
{
  return new OpenraveRobotMemoryInterface::ConstructSceneMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
OpenraveRobotMemoryInterface::message_valid(const Message *message) const
{
  const ConstructSceneMessage *m0 = dynamic_cast<const ConstructSceneMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(OpenraveRobotMemoryInterface)
/// @endcond


} // end namespace fawkes
