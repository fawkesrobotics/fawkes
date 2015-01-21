
/***************************************************************************
 *  JointInterface.cpp - Fawkes BlackBoard Interface - JointInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2013  Till Hofmann
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

#include <interfaces/JointInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class JointInterface <interfaces/JointInterface.h>
 * JointInterface Fawkes BlackBoard Interface.
 * 
      Storage for a single joint state.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
JointInterface::JointInterface() : Interface()
{
  data_size = sizeof(JointInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (JointInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_FLOAT, "position", 1, &data->position);
  add_fieldinfo(IFT_FLOAT, "velocity", 1, &data->velocity);
  unsigned char tmp_hash[] = {0xd2, 0x74, 0x1b, 0x6a, 0x5b, 0xf, 0xa9, 0xe1, 0xb0, 0xa8, 0x47, 0x84, 0x6f, 0x8f, 0x1c, 0xab};
  set_hash(tmp_hash);
}

/** Destructor */
JointInterface::~JointInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get position value.
 * 
      The joint's position in rad.
    
 * @return position value
 */
float
JointInterface::position() const
{
  return data->position;
}

/** Get maximum length of position value.
 * @return length of position value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JointInterface::maxlenof_position() const
{
  return 1;
}

/** Set position value.
 * 
      The joint's position in rad.
    
 * @param new_position new position value
 */
void
JointInterface::set_position(const float new_position)
{
  data->position = new_position;
  data_changed = true;
}

/** Get velocity value.
 * 
      The joint's velocity in rad/s.
    
 * @return velocity value
 */
float
JointInterface::velocity() const
{
  return data->velocity;
}

/** Get maximum length of velocity value.
 * @return length of velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JointInterface::maxlenof_velocity() const
{
  return 1;
}

/** Set velocity value.
 * 
      The joint's velocity in rad/s.
    
 * @param new_velocity new velocity value
 */
void
JointInterface::set_velocity(const float new_velocity)
{
  data->velocity = new_velocity;
  data_changed = true;
}

/* =========== message create =========== */
Message *
JointInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
JointInterface::copy_values(const Interface *other)
{
  const JointInterface *oi = dynamic_cast<const JointInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(JointInterface_data_t));
}

const char *
JointInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
JointInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(JointInterface)
/// @endcond


} // end namespace fawkes
