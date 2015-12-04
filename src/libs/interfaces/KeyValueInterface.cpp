
/***************************************************************************
 *  KeyValueInterface.cpp - Fawkes BlackBoard Interface - KeyValueInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2015  Gesche Gierse
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

#include <interfaces/KeyValueInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class KeyValueInterface <interfaces/KeyValueInterface.h>
 * KeyValueInterface Fawkes BlackBoard Interface.
 * Key-Value interface. Use this to publish Key-Value based information, if you do not want to create a new interface type for the data. This interface can be used for different kind of data, but should only contain one value at a time. Set the value_type field to represent which kind of value should be transported (e.g. TYPE_INT for integer) and fill the data in the correct value field (e.g. value_int).
 * @ingroup FawkesInterfaces
 */



/** Constructor */
KeyValueInterface::KeyValueInterface() : Interface()
{
  data_size = sizeof(KeyValueInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (KeyValueInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_ValueType[(int)TypeStr] = "TypeStr";
  enum_map_ValueType[(int)TypeInt] = "TypeInt";
  enum_map_ValueType[(int)TypeUint] = "TypeUint";
  enum_map_ValueType[(int)TypeBool] = "TypeBool";
  enum_map_ValueType[(int)TypeByte] = "TypeByte";
  enum_map_ValueType[(int)TypeFloat] = "TypeFloat";
  add_fieldinfo(IFT_STRING, "key", 32, data->key);
  add_fieldinfo(IFT_ENUM, "value_type", 1, &data->value_type, "ValueType", &enum_map_ValueType);
  add_fieldinfo(IFT_STRING, "value_string", 32, data->value_string);
  add_fieldinfo(IFT_UINT32, "value_uint", 1, &data->value_uint);
  add_fieldinfo(IFT_INT32, "value_int", 1, &data->value_int);
  add_fieldinfo(IFT_BOOL, "value_bool", 1, &data->value_bool);
  add_fieldinfo(IFT_BYTE, "value_byte", 1, &data->value_byte);
  add_fieldinfo(IFT_FLOAT, "value_float", 1, &data->value_float);
  unsigned char tmp_hash[] = {0xf1, 0x89, 0x81, 0x4f, 0xb9, 0x6e, 0x5c, 0xc8, 0x78, 0x90, 0x1a, 0x10, 0xdb, 0xa9, 0xa0, 0x52};
  set_hash(tmp_hash);
}

/** Destructor */
KeyValueInterface::~KeyValueInterface()
{
  free(data_ptr);
}
/** Convert ValueType constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
KeyValueInterface::tostring_ValueType(ValueType value) const
{
  switch (value) {
  case TypeStr: return "TypeStr";
  case TypeInt: return "TypeInt";
  case TypeUint: return "TypeUint";
  case TypeBool: return "TypeBool";
  case TypeByte: return "TypeByte";
  case TypeFloat: return "TypeFloat";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get key value.
 * The key entry
 * @return key value
 */
char *
KeyValueInterface::key() const
{
  return data->key;
}

/** Get maximum length of key value.
 * @return length of key value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KeyValueInterface::maxlenof_key() const
{
  return 32;
}

/** Set key value.
 * The key entry
 * @param new_key new key value
 */
void
KeyValueInterface::set_key(const char * new_key)
{
  strncpy(data->key, new_key, sizeof(data->key));
  data_changed = true;
}

/** Get value_type value.
 * The type of the value entry.
 * @return value_type value
 */
KeyValueInterface::ValueType
KeyValueInterface::value_type() const
{
  return (KeyValueInterface::ValueType)data->value_type;
}

/** Get maximum length of value_type value.
 * @return length of value_type value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KeyValueInterface::maxlenof_value_type() const
{
  return 1;
}

/** Set value_type value.
 * The type of the value entry.
 * @param new_value_type new value_type value
 */
void
KeyValueInterface::set_value_type(const ValueType new_value_type)
{
  data->value_type = new_value_type;
  data_changed = true;
}

/** Get value_string value.
 * Value with type string
 * @return value_string value
 */
char *
KeyValueInterface::value_string() const
{
  return data->value_string;
}

/** Get maximum length of value_string value.
 * @return length of value_string value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KeyValueInterface::maxlenof_value_string() const
{
  return 32;
}

/** Set value_string value.
 * Value with type string
 * @param new_value_string new value_string value
 */
void
KeyValueInterface::set_value_string(const char * new_value_string)
{
  strncpy(data->value_string, new_value_string, sizeof(data->value_string));
  data_changed = true;
}

/** Get value_uint value.
 * Value with type uint32
 * @return value_uint value
 */
uint32_t
KeyValueInterface::value_uint() const
{
  return data->value_uint;
}

/** Get maximum length of value_uint value.
 * @return length of value_uint value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KeyValueInterface::maxlenof_value_uint() const
{
  return 1;
}

/** Set value_uint value.
 * Value with type uint32
 * @param new_value_uint new value_uint value
 */
void
KeyValueInterface::set_value_uint(const uint32_t new_value_uint)
{
  data->value_uint = new_value_uint;
  data_changed = true;
}

/** Get value_int value.
 * Value with type integer
 * @return value_int value
 */
int32_t
KeyValueInterface::value_int() const
{
  return data->value_int;
}

/** Get maximum length of value_int value.
 * @return length of value_int value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KeyValueInterface::maxlenof_value_int() const
{
  return 1;
}

/** Set value_int value.
 * Value with type integer
 * @param new_value_int new value_int value
 */
void
KeyValueInterface::set_value_int(const int32_t new_value_int)
{
  data->value_int = new_value_int;
  data_changed = true;
}

/** Get value_bool value.
 *  Value with type Bool
 * @return value_bool value
 */
bool
KeyValueInterface::is_value_bool() const
{
  return data->value_bool;
}

/** Get maximum length of value_bool value.
 * @return length of value_bool value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KeyValueInterface::maxlenof_value_bool() const
{
  return 1;
}

/** Set value_bool value.
 *  Value with type Bool
 * @param new_value_bool new value_bool value
 */
void
KeyValueInterface::set_value_bool(const bool new_value_bool)
{
  data->value_bool = new_value_bool;
  data_changed = true;
}

/** Get value_byte value.
 * Value with type byte
 * @return value_byte value
 */
uint8_t
KeyValueInterface::value_byte() const
{
  return data->value_byte;
}

/** Get maximum length of value_byte value.
 * @return length of value_byte value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KeyValueInterface::maxlenof_value_byte() const
{
  return 1;
}

/** Set value_byte value.
 * Value with type byte
 * @param new_value_byte new value_byte value
 */
void
KeyValueInterface::set_value_byte(const uint8_t new_value_byte)
{
  data->value_byte = new_value_byte;
  data_changed = true;
}

/** Get value_float value.
 * Value with type float
 * @return value_float value
 */
float
KeyValueInterface::value_float() const
{
  return data->value_float;
}

/** Get maximum length of value_float value.
 * @return length of value_float value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KeyValueInterface::maxlenof_value_float() const
{
  return 1;
}

/** Set value_float value.
 * Value with type float
 * @param new_value_float new value_float value
 */
void
KeyValueInterface::set_value_float(const float new_value_float)
{
  data->value_float = new_value_float;
  data_changed = true;
}

/* =========== message create =========== */
Message *
KeyValueInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
KeyValueInterface::copy_values(const Interface *other)
{
  const KeyValueInterface *oi = dynamic_cast<const KeyValueInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(KeyValueInterface_data_t));
}

const char *
KeyValueInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "ValueType") == 0) {
    return tostring_ValueType((ValueType)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
KeyValueInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(KeyValueInterface)
/// @endcond


} // end namespace fawkes
