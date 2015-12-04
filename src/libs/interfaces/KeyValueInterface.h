
/***************************************************************************
 *  KeyValueInterface.h - Fawkes BlackBoard Interface - KeyValueInterface
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

#ifndef __INTERFACES_KEYVALUEINTERFACE_H_
#define __INTERFACES_KEYVALUEINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class KeyValueInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(KeyValueInterface)
 /// @endcond
 public:
  /* constants */

  /** Indicator of current o. */
  typedef enum {
    TypeStr /**< The value to be transported is of type string. */,
    TypeInt /**< The value to be transported is of type integer. */,
    TypeUint /**< The value to be transported is of type unsigned integer. */,
    TypeBool /**< The value to be transported is of type boolean. */,
    TypeByte /**< The value to be transported is of type byte. */,
    TypeFloat /**< The value to be transported is of type float. */
  } ValueType;
  const char * tostring_ValueType(ValueType value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    char key[32]; /**< The key entry */
    int32_t value_type; /**< The type of the value entry. */
    char value_string[32]; /**< Value with type string */
    uint32_t value_uint; /**< Value with type uint32 */
    int32_t value_int; /**< Value with type integer */
    bool value_bool; /**<  Value with type Bool */
    uint8_t value_byte; /**< Value with type byte */
    float value_float; /**< Value with type float */
  } KeyValueInterface_data_t;
#pragma pack(pop)

  KeyValueInterface_data_t *data;

  interface_enum_map_t enum_map_ValueType;
 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  KeyValueInterface();
  ~KeyValueInterface();

 public:
  /* Methods */
  char * key() const;
  void set_key(const char * new_key);
  size_t maxlenof_key() const;
  ValueType value_type() const;
  void set_value_type(const ValueType new_value_type);
  size_t maxlenof_value_type() const;
  char * value_string() const;
  void set_value_string(const char * new_value_string);
  size_t maxlenof_value_string() const;
  uint32_t value_uint() const;
  void set_value_uint(const uint32_t new_value_uint);
  size_t maxlenof_value_uint() const;
  int32_t value_int() const;
  void set_value_int(const int32_t new_value_int);
  size_t maxlenof_value_int() const;
  bool is_value_bool() const;
  void set_value_bool(const bool new_value_bool);
  size_t maxlenof_value_bool() const;
  uint8_t value_byte() const;
  void set_value_byte(const uint8_t new_value_byte);
  size_t maxlenof_value_byte() const;
  float value_float() const;
  void set_value_float(const float new_value_float);
  size_t maxlenof_value_float() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
