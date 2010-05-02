
/***************************************************************************
 *  LocalizerControlInterface.cpp - Fawkes BlackBoard Interface - LocalizerControlInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2009  Daniel Beck
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

#include <interfaces/LocalizerControlInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class LocalizerControlInterface <interfaces/LocalizerControlInterface.h>
 * LocalizerControlInterface Fawkes BlackBoard Interface.
 * 
      This interface allows observe the current status of the a
      localizer as well as sending it commands (eg., reset,
      re-position, etc.)
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
LocalizerControlInterface::LocalizerControlInterface() : Interface()
{
  data_size = sizeof(LocalizerControlInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (LocalizerControlInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "map_name", 30, data->map_name);
  add_messageinfo("ResetMessage");
  unsigned char tmp_hash[] = {0xa4, 0xe8, 0x69, 0x11, 0x29, 0x30, 0xf2, 0xcb, 0xe5, 0xf4, 00, 0x35, 0x19, 0x58, 0x54, 0xfb};
  set_hash(tmp_hash);
}

/** Destructor */
LocalizerControlInterface::~LocalizerControlInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get map_name value.
 * The name of the current
    map
 * @return map_name value
 */
char *
LocalizerControlInterface::map_name() const
{
  return data->map_name;
}

/** Get maximum length of map_name value.
 * @return length of map_name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LocalizerControlInterface::maxlenof_map_name() const
{
  return 30;
}

/** Set map_name value.
 * The name of the current
    map
 * @param new_map_name new map_name value
 */
void
LocalizerControlInterface::set_map_name(const char * new_map_name)
{
  strncpy(data->map_name, new_map_name, sizeof(data->map_name));
  data_changed = true;
}

/* =========== message create =========== */
Message *
LocalizerControlInterface::create_message(const char *type) const
{
  if ( strncmp("ResetMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ResetMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
LocalizerControlInterface::copy_values(const Interface *other)
{
  const LocalizerControlInterface *oi = dynamic_cast<const LocalizerControlInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(LocalizerControlInterface_data_t));
}

const char *
LocalizerControlInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class LocalizerControlInterface::ResetMessage <interfaces/LocalizerControlInterface.h>
 * ResetMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_ori initial value for ori
 * @param ini_variance initial value for variance
 */
LocalizerControlInterface::ResetMessage::ResetMessage(const float ini_x, const float ini_y, const float ini_ori, const float ini_variance) : Message("ResetMessage")
{
  data_size = sizeof(ResetMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ResetMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->ori = ini_ori;
  data->variance = ini_variance;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "ori", 1, &data->ori);
  add_fieldinfo(IFT_FLOAT, "variance", 1, &data->variance);
}
/** Constructor */
LocalizerControlInterface::ResetMessage::ResetMessage() : Message("ResetMessage")
{
  data_size = sizeof(ResetMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ResetMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "ori", 1, &data->ori);
  add_fieldinfo(IFT_FLOAT, "variance", 1, &data->variance);
}

/** Destructor */
LocalizerControlInterface::ResetMessage::~ResetMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
LocalizerControlInterface::ResetMessage::ResetMessage(const ResetMessage *m) : Message("ResetMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ResetMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * The new initial x-coordinate.
 * @return x value
 */
float
LocalizerControlInterface::ResetMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LocalizerControlInterface::ResetMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * The new initial x-coordinate.
 * @param new_x new x value
 */
void
LocalizerControlInterface::ResetMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * The new initial x-coordinate.
 * @return y value
 */
float
LocalizerControlInterface::ResetMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LocalizerControlInterface::ResetMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * The new initial x-coordinate.
 * @param new_y new y value
 */
void
LocalizerControlInterface::ResetMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get ori value.
 * The new initial orientation.
 * @return ori value
 */
float
LocalizerControlInterface::ResetMessage::ori() const
{
  return data->ori;
}

/** Get maximum length of ori value.
 * @return length of ori value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LocalizerControlInterface::ResetMessage::maxlenof_ori() const
{
  return 1;
}

/** Set ori value.
 * The new initial orientation.
 * @param new_ori new ori value
 */
void
LocalizerControlInterface::ResetMessage::set_ori(const float new_ori)
{
  data->ori = new_ori;
}

/** Get variance value.
 * The variance for the reset position.
 * @return variance value
 */
float
LocalizerControlInterface::ResetMessage::variance() const
{
  return data->variance;
}

/** Get maximum length of variance value.
 * @return length of variance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LocalizerControlInterface::ResetMessage::maxlenof_variance() const
{
  return 1;
}

/** Set variance value.
 * The variance for the reset position.
 * @param new_variance new variance value
 */
void
LocalizerControlInterface::ResetMessage::set_variance(const float new_variance)
{
  data->variance = new_variance;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
LocalizerControlInterface::ResetMessage::clone() const
{
  return new LocalizerControlInterface::ResetMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
LocalizerControlInterface::message_valid(const Message *message) const
{
  const ResetMessage *m0 = dynamic_cast<const ResetMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(LocalizerControlInterface)
/// @endcond


} // end namespace fawkes
