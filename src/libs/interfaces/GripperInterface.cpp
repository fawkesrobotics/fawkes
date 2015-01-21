
/***************************************************************************
 *  GripperInterface.cpp - Fawkes BlackBoard Interface - GripperInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2013  Sebastian Reuter
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

#include <interfaces/GripperInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class GripperInterface <interfaces/GripperInterface.h>
 * GripperInterface Fawkes BlackBoard Interface.
 * 
      This interface provides support for a simple gripper actuator.
      It has been used with the Robotino Gripper.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
GripperInterface::GripperInterface() : Interface()
{
  data_size = sizeof(GripperInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (GripperInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_GripperState[(int)OPEN] = "OPEN";
  enum_map_GripperState[(int)CLOSED] = "CLOSED";
  add_fieldinfo(IFT_ENUM, "gripper_state", 1, &data->gripper_state, "GripperState", &enum_map_GripperState);
  add_messageinfo("OpenGripperMessage");
  add_messageinfo("CloseGripperMessage");
  unsigned char tmp_hash[] = {0xf8, 0xd6, 0x88, 0xb4, 0xfc, 0xfa, 0x1f, 0x1b, 0x20, 0x9f, 0xc, 0xd, 0x81, 0x3c, 0xba, 0xdf};
  set_hash(tmp_hash);
}

/** Destructor */
GripperInterface::~GripperInterface()
{
  free(data_ptr);
}
/** Convert GripperState constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
GripperInterface::tostring_GripperState(GripperState value) const
{
  switch (value) {
  case OPEN: return "OPEN";
  case CLOSED: return "CLOSED";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get gripper_state value.
 * 
      The current state of the gripper.
    
 * @return gripper_state value
 */
GripperInterface::GripperState
GripperInterface::gripper_state() const
{
  return (GripperInterface::GripperState)data->gripper_state;
}

/** Get maximum length of gripper_state value.
 * @return length of gripper_state value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GripperInterface::maxlenof_gripper_state() const
{
  return 1;
}

/** Set gripper_state value.
 * 
      The current state of the gripper.
    
 * @param new_gripper_state new gripper_state value
 */
void
GripperInterface::set_gripper_state(const GripperState new_gripper_state)
{
  data->gripper_state = new_gripper_state;
  data_changed = true;
}

/* =========== message create =========== */
Message *
GripperInterface::create_message(const char *type) const
{
  if ( strncmp("OpenGripperMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new OpenGripperMessage();
  } else if ( strncmp("CloseGripperMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CloseGripperMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
GripperInterface::copy_values(const Interface *other)
{
  const GripperInterface *oi = dynamic_cast<const GripperInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(GripperInterface_data_t));
}

const char *
GripperInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "GripperState") == 0) {
    return tostring_GripperState((GripperState)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class GripperInterface::OpenGripperMessage <interfaces/GripperInterface.h>
 * OpenGripperMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
GripperInterface::OpenGripperMessage::OpenGripperMessage() : Message("OpenGripperMessage")
{
  data_size = sizeof(OpenGripperMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (OpenGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_GripperState[(int)OPEN] = "OPEN";
  enum_map_GripperState[(int)CLOSED] = "CLOSED";
}

/** Destructor */
GripperInterface::OpenGripperMessage::~OpenGripperMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
GripperInterface::OpenGripperMessage::OpenGripperMessage(const OpenGripperMessage *m) : Message("OpenGripperMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (OpenGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
GripperInterface::OpenGripperMessage::clone() const
{
  return new GripperInterface::OpenGripperMessage(this);
}
/** @class GripperInterface::CloseGripperMessage <interfaces/GripperInterface.h>
 * CloseGripperMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
GripperInterface::CloseGripperMessage::CloseGripperMessage() : Message("CloseGripperMessage")
{
  data_size = sizeof(CloseGripperMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CloseGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_GripperState[(int)OPEN] = "OPEN";
  enum_map_GripperState[(int)CLOSED] = "CLOSED";
}

/** Destructor */
GripperInterface::CloseGripperMessage::~CloseGripperMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
GripperInterface::CloseGripperMessage::CloseGripperMessage(const CloseGripperMessage *m) : Message("CloseGripperMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CloseGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
GripperInterface::CloseGripperMessage::clone() const
{
  return new GripperInterface::CloseGripperMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
GripperInterface::message_valid(const Message *message) const
{
  const OpenGripperMessage *m0 = dynamic_cast<const OpenGripperMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const CloseGripperMessage *m1 = dynamic_cast<const CloseGripperMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(GripperInterface)
/// @endcond


} // end namespace fawkes
