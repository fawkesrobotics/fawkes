
/***************************************************************************
 *  LaserClusterInterface.cpp - Fawkes BlackBoard Interface - LaserClusterInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2013  Tim Niemueller
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

#include <interfaces/LaserClusterInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class LaserClusterInterface <interfaces/LaserClusterInterface.h>
 * LaserClusterInterface Fawkes BlackBoard Interface.
 * Laser cluster parameterization.
 * @ingroup FawkesInterfaces
 */



/** Constructor */
LaserClusterInterface::LaserClusterInterface() : Interface()
{
  data_size = sizeof(LaserClusterInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (LaserClusterInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_FLOAT, "max_x", 1, &data->max_x);
  add_messageinfo("SetMaxXMessage");
  unsigned char tmp_hash[] = {0xa6, 0x6a, 0xdb, 0xa, 0x3, 0x63, 0x9a, 0x99, 0x95, 0x1f, 0x45, 0x4b, 0xb8, 0xe4, 0xb2, 0x33};
  set_hash(tmp_hash);
}

/** Destructor */
LaserClusterInterface::~LaserClusterInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get max_x value.
 * Maximum distance in X coordinate
    of sensor frame.
 * @return max_x value
 */
float
LaserClusterInterface::max_x() const
{
  return data->max_x;
}

/** Get maximum length of max_x value.
 * @return length of max_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LaserClusterInterface::maxlenof_max_x() const
{
  return 1;
}

/** Set max_x value.
 * Maximum distance in X coordinate
    of sensor frame.
 * @param new_max_x new max_x value
 */
void
LaserClusterInterface::set_max_x(const float new_max_x)
{
  data->max_x = new_max_x;
  data_changed = true;
}

/* =========== message create =========== */
Message *
LaserClusterInterface::create_message(const char *type) const
{
  if ( strncmp("SetMaxXMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetMaxXMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
LaserClusterInterface::copy_values(const Interface *other)
{
  const LaserClusterInterface *oi = dynamic_cast<const LaserClusterInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(LaserClusterInterface_data_t));
}

const char *
LaserClusterInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class LaserClusterInterface::SetMaxXMessage <interfaces/LaserClusterInterface.h>
 * SetMaxXMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_max_x initial value for max_x
 */
LaserClusterInterface::SetMaxXMessage::SetMaxXMessage(const float ini_max_x) : Message("SetMaxXMessage")
{
  data_size = sizeof(SetMaxXMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMaxXMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->max_x = ini_max_x;
  add_fieldinfo(IFT_FLOAT, "max_x", 1, &data->max_x);
}
/** Constructor */
LaserClusterInterface::SetMaxXMessage::SetMaxXMessage() : Message("SetMaxXMessage")
{
  data_size = sizeof(SetMaxXMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMaxXMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "max_x", 1, &data->max_x);
}

/** Destructor */
LaserClusterInterface::SetMaxXMessage::~SetMaxXMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
LaserClusterInterface::SetMaxXMessage::SetMaxXMessage(const SetMaxXMessage *m) : Message("SetMaxXMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetMaxXMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get max_x value.
 * Maximum distance in X coordinate
    of sensor frame.
 * @return max_x value
 */
float
LaserClusterInterface::SetMaxXMessage::max_x() const
{
  return data->max_x;
}

/** Get maximum length of max_x value.
 * @return length of max_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LaserClusterInterface::SetMaxXMessage::maxlenof_max_x() const
{
  return 1;
}

/** Set max_x value.
 * Maximum distance in X coordinate
    of sensor frame.
 * @param new_max_x new max_x value
 */
void
LaserClusterInterface::SetMaxXMessage::set_max_x(const float new_max_x)
{
  data->max_x = new_max_x;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
LaserClusterInterface::SetMaxXMessage::clone() const
{
  return new LaserClusterInterface::SetMaxXMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
LaserClusterInterface::message_valid(const Message *message) const
{
  const SetMaxXMessage *m0 = dynamic_cast<const SetMaxXMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(LaserClusterInterface)
/// @endcond


} // end namespace fawkes
