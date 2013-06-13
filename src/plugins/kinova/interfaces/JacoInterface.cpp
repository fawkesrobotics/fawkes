
/***************************************************************************
 *  JacoInterface.cpp - Fawkes BlackBoard Interface - JacoInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2013  Bahram Maleki-Fard
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

#include <interfaces/JacoInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class JacoInterface <interfaces/JacoInterface.h>
 * JacoInterface Fawkes BlackBoard Interface.
 * 
      Interface providing access to a Kinova Jaco arm.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
JacoInterface::JacoInterface() : Interface()
{
  data_size = sizeof(JacoInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (JacoInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_BOOL, "connected", 1, &data->connected);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
  add_fieldinfo(IFT_FLOAT, "euler1", 1, &data->euler1);
  add_fieldinfo(IFT_FLOAT, "euler2", 1, &data->euler2);
  add_fieldinfo(IFT_FLOAT, "euler3", 1, &data->euler3);
  add_fieldinfo(IFT_FLOAT, "joints", 6, &data->joints);
  add_fieldinfo(IFT_FLOAT, "finger1", 1, &data->finger1);
  add_fieldinfo(IFT_FLOAT, "finger2", 1, &data->finger2);
  add_fieldinfo(IFT_FLOAT, "finger3", 1, &data->finger3);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  unsigned char tmp_hash[] = {0x3, 0x15, 0x3d, 0x5a, 0xc0, 0xce, 0xc4, 0x41, 0xd6, 0x68, 0x32, 0xe1, 0x86, 0x5b, 0xce, 0x90};
  set_hash(tmp_hash);
}

/** Destructor */
JacoInterface::~JacoInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get connected value.
 * Is JacoArm connected/ready?
 * @return connected value
 */
bool
JacoInterface::is_connected() const
{
  return data->connected;
}

/** Get maximum length of connected value.
 * @return length of connected value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_connected() const
{
  return 1;
}

/** Set connected value.
 * Is JacoArm connected/ready?
 * @param new_connected new connected value
 */
void
JacoInterface::set_connected(const bool new_connected)
{
  data->connected = new_connected;
  data_changed = true;
}

/** Get x value.
 * X-Coordinate of tool translation.
 * @return x value
 */
float
JacoInterface::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X-Coordinate of tool translation.
 * @param new_x new x value
 */
void
JacoInterface::set_x(const float new_x)
{
  data->x = new_x;
  data_changed = true;
}

/** Get y value.
 * Y-Coordinate op tool translation.
 * @return y value
 */
float
JacoInterface::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y-Coordinate op tool translation.
 * @param new_y new y value
 */
void
JacoInterface::set_y(const float new_y)
{
  data->y = new_y;
  data_changed = true;
}

/** Get z value.
 * Z-Coordinate of tool translation.
 * @return z value
 */
float
JacoInterface::z() const
{
  return data->z;
}

/** Get maximum length of z value.
 * @return length of z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_z() const
{
  return 1;
}

/** Set z value.
 * Z-Coordinate of tool translation.
 * @param new_z new z value
 */
void
JacoInterface::set_z(const float new_z)
{
  data->z = new_z;
  data_changed = true;
}

/** Get euler1 value.
 * 1st Euler angle of tool rotation.
 * @return euler1 value
 */
float
JacoInterface::euler1() const
{
  return data->euler1;
}

/** Get maximum length of euler1 value.
 * @return length of euler1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_euler1() const
{
  return 1;
}

/** Set euler1 value.
 * 1st Euler angle of tool rotation.
 * @param new_euler1 new euler1 value
 */
void
JacoInterface::set_euler1(const float new_euler1)
{
  data->euler1 = new_euler1;
  data_changed = true;
}

/** Get euler2 value.
 * 2nd Euler angle of tool rotation.
 * @return euler2 value
 */
float
JacoInterface::euler2() const
{
  return data->euler2;
}

/** Get maximum length of euler2 value.
 * @return length of euler2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_euler2() const
{
  return 1;
}

/** Set euler2 value.
 * 2nd Euler angle of tool rotation.
 * @param new_euler2 new euler2 value
 */
void
JacoInterface::set_euler2(const float new_euler2)
{
  data->euler2 = new_euler2;
  data_changed = true;
}

/** Get euler3 value.
 * 3rd Euler angle of tool rotation.
 * @return euler3 value
 */
float
JacoInterface::euler3() const
{
  return data->euler3;
}

/** Get maximum length of euler3 value.
 * @return length of euler3 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_euler3() const
{
  return 1;
}

/** Set euler3 value.
 * 3rd Euler angle of tool rotation.
 * @param new_euler3 new euler3 value
 */
void
JacoInterface::set_euler3(const float new_euler3)
{
  data->euler3 = new_euler3;
  data_changed = true;
}

/** Get joints value.
 * Angle values of joints
 * @return joints value
 */
float *
JacoInterface::joints() const
{
  return data->joints;
}

/** Get joints value at given index.
 * Angle values of joints
 * @param index index of value
 * @return joints value
 * @exception Exception thrown if index is out of bounds
 */
float
JacoInterface::joints(unsigned int index) const
{
  if (index > 6) {
    throw Exception("Index value %u out of bounds (0..6)", index);
  }
  return data->joints[index];
}

/** Get maximum length of joints value.
 * @return length of joints value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_joints() const
{
  return 6;
}

/** Set joints value.
 * Angle values of joints
 * @param new_joints new joints value
 */
void
JacoInterface::set_joints(const float * new_joints)
{
  memcpy(data->joints, new_joints, sizeof(float) * 6);
  data_changed = true;
}

/** Set joints value at given index.
 * Angle values of joints
 * @param new_joints new joints value
 * @param index index for of the value
 */
void
JacoInterface::set_joints(unsigned int index, const float new_joints)
{
  if (index > 6) {
    throw Exception("Index value %u out of bounds (0..6)", index);
  }
  data->joints[index] = new_joints;
  data_changed = true;
}
/** Get finger1 value.
 * Angular value of finger 1.
 * @return finger1 value
 */
float
JacoInterface::finger1() const
{
  return data->finger1;
}

/** Get maximum length of finger1 value.
 * @return length of finger1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_finger1() const
{
  return 1;
}

/** Set finger1 value.
 * Angular value of finger 1.
 * @param new_finger1 new finger1 value
 */
void
JacoInterface::set_finger1(const float new_finger1)
{
  data->finger1 = new_finger1;
  data_changed = true;
}

/** Get finger2 value.
 * Angular value of finger 2.
 * @return finger2 value
 */
float
JacoInterface::finger2() const
{
  return data->finger2;
}

/** Get maximum length of finger2 value.
 * @return length of finger2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_finger2() const
{
  return 1;
}

/** Set finger2 value.
 * Angular value of finger 2.
 * @param new_finger2 new finger2 value
 */
void
JacoInterface::set_finger2(const float new_finger2)
{
  data->finger2 = new_finger2;
  data_changed = true;
}

/** Get finger3 value.
 * Angular value of finger 3.
 * @return finger3 value
 */
float
JacoInterface::finger3() const
{
  return data->finger3;
}

/** Get maximum length of finger3 value.
 * @return length of finger3 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_finger3() const
{
  return 1;
}

/** Set finger3 value.
 * Angular value of finger 3.
 * @param new_finger3 new finger3 value
 */
void
JacoInterface::set_finger3(const float new_finger3)
{
  data->finger3 = new_finger3;
  data_changed = true;
}

/** Get msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @return msgid value
 */
uint32_t
JacoInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @param new_msgid new msgid value
 */
void
JacoInterface::set_msgid(const uint32_t new_msgid)
{
  data->msgid = new_msgid;
  data_changed = true;
}

/** Get final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @return final value
 */
bool
JacoInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @param new_final new final value
 */
void
JacoInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/* =========== message create =========== */
Message *
JacoInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
JacoInterface::copy_values(const Interface *other)
{
  const JacoInterface *oi = dynamic_cast<const JacoInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(JacoInterface_data_t));
}

const char *
JacoInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
JacoInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(JacoInterface)
/// @endcond


} // end namespace fawkes
