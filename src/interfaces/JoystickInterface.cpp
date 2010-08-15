
/***************************************************************************
 *  JoystickInterface.cpp - Fawkes BlackBoard Interface - JoystickInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
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

#include <interfaces/JoystickInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class JoystickInterface <interfaces/JoystickInterface.h>
 * JoystickInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to a joystick. It provides up to
      five axes, where each has a X and a Y value between -1.0 and 1.0.
      Up to 32 buttons are support via an uint32 bit field.
    
 * @ingroup FawkesInterfaces
 */


/** BUTTON_1 constant */
const uint32_t JoystickInterface::BUTTON_1 = 1u;
/** BUTTON_2 constant */
const uint32_t JoystickInterface::BUTTON_2 = 2u;
/** BUTTON_3 constant */
const uint32_t JoystickInterface::BUTTON_3 = 4u;
/** BUTTON_4 constant */
const uint32_t JoystickInterface::BUTTON_4 = 8u;
/** BUTTON_5 constant */
const uint32_t JoystickInterface::BUTTON_5 = 16u;
/** BUTTON_6 constant */
const uint32_t JoystickInterface::BUTTON_6 = 32u;
/** BUTTON_7 constant */
const uint32_t JoystickInterface::BUTTON_7 = 64u;
/** BUTTON_8 constant */
const uint32_t JoystickInterface::BUTTON_8 = 128u;
/** BUTTON_9 constant */
const uint32_t JoystickInterface::BUTTON_9 = 256u;
/** BUTTON_10 constant */
const uint32_t JoystickInterface::BUTTON_10 = 512u;
/** BUTTON_11 constant */
const uint32_t JoystickInterface::BUTTON_11 = 1024u;
/** BUTTON_12 constant */
const uint32_t JoystickInterface::BUTTON_12 = 2048u;
/** BUTTON_13 constant */
const uint32_t JoystickInterface::BUTTON_13 = 4096u;
/** BUTTON_14 constant */
const uint32_t JoystickInterface::BUTTON_14 = 8192u;
/** BUTTON_15 constant */
const uint32_t JoystickInterface::BUTTON_15 = 16384u;
/** BUTTON_16 constant */
const uint32_t JoystickInterface::BUTTON_16 = 32768u;
/** BUTTON_17 constant */
const uint32_t JoystickInterface::BUTTON_17 = 65536u;
/** BUTTON_18 constant */
const uint32_t JoystickInterface::BUTTON_18 = 131072u;
/** BUTTON_19 constant */
const uint32_t JoystickInterface::BUTTON_19 = 262144u;
/** BUTTON_20 constant */
const uint32_t JoystickInterface::BUTTON_20 = 524288u;
/** BUTTON_21 constant */
const uint32_t JoystickInterface::BUTTON_21 = 1048576u;
/** BUTTON_22 constant */
const uint32_t JoystickInterface::BUTTON_22 = 2097152u;
/** BUTTON_23 constant */
const uint32_t JoystickInterface::BUTTON_23 = 4194304u;
/** BUTTON_24 constant */
const uint32_t JoystickInterface::BUTTON_24 = 8388608u;
/** BUTTON_25 constant */
const uint32_t JoystickInterface::BUTTON_25 = 16777216u;
/** BUTTON_26 constant */
const uint32_t JoystickInterface::BUTTON_26 = 33554432u;
/** BUTTON_27 constant */
const uint32_t JoystickInterface::BUTTON_27 = 67108864u;
/** BUTTON_28 constant */
const uint32_t JoystickInterface::BUTTON_28 = 134217728u;
/** BUTTON_29 constant */
const uint32_t JoystickInterface::BUTTON_29 = 268435456u;
/** BUTTON_30 constant */
const uint32_t JoystickInterface::BUTTON_30 = 536870912u;
/** BUTTON_31 constant */
const uint32_t JoystickInterface::BUTTON_31 = 1073741824u;
/** BUTTON_32 constant */
const uint32_t JoystickInterface::BUTTON_32 = 2147483648u;

/** Constructor */
JoystickInterface::JoystickInterface() : Interface()
{
  data_size = sizeof(JoystickInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (JoystickInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_BYTE, "num_axes", 1, &data->num_axes);
  add_fieldinfo(IFT_BYTE, "num_buttons", 1, &data->num_buttons);
  add_fieldinfo(IFT_UINT32, "pressed_buttons", 1, &data->pressed_buttons);
  add_fieldinfo(IFT_FLOAT, "axis_x", 4, &data->axis_x);
  add_fieldinfo(IFT_FLOAT, "axis_y", 4, &data->axis_y);
  unsigned char tmp_hash[] = {0x20, 0xe5, 0x9c, 0x19, 0x6e, 0xd2, 0xcf, 0xcc, 0xf2, 0x5d, 0x70, 0x88, 0x52, 0x66, 0x7a, 0x1e};
  set_hash(tmp_hash);
}

/** Destructor */
JoystickInterface::~JoystickInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get num_axes value.
 * 
      The number of axes of this joystick
    
 * @return num_axes value
 */
uint8_t
JoystickInterface::num_axes() const
{
  return data->num_axes;
}

/** Get maximum length of num_axes value.
 * @return length of num_axes value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::maxlenof_num_axes() const
{
  return 1;
}

/** Set num_axes value.
 * 
      The number of axes of this joystick
    
 * @param new_num_axes new num_axes value
 */
void
JoystickInterface::set_num_axes(const uint8_t new_num_axes)
{
  data->num_axes = new_num_axes;
  data_changed = true;
}

/** Get num_buttons value.
 * 
      The number of buttons of this joystick.
    
 * @return num_buttons value
 */
uint8_t
JoystickInterface::num_buttons() const
{
  return data->num_buttons;
}

/** Get maximum length of num_buttons value.
 * @return length of num_buttons value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::maxlenof_num_buttons() const
{
  return 1;
}

/** Set num_buttons value.
 * 
      The number of buttons of this joystick.
    
 * @param new_num_buttons new num_buttons value
 */
void
JoystickInterface::set_num_buttons(const uint8_t new_num_buttons)
{
  data->num_buttons = new_num_buttons;
  data_changed = true;
}

/** Get pressed_buttons value.
 * 
      A bit field of enabled buttons. For each currently clicked button the
      corresponding bit is set to 1. Use the BUTTON_* constants for bit-wise
      comparisons.
    
 * @return pressed_buttons value
 */
uint32_t
JoystickInterface::pressed_buttons() const
{
  return data->pressed_buttons;
}

/** Get maximum length of pressed_buttons value.
 * @return length of pressed_buttons value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::maxlenof_pressed_buttons() const
{
  return 1;
}

/** Set pressed_buttons value.
 * 
      A bit field of enabled buttons. For each currently clicked button the
      corresponding bit is set to 1. Use the BUTTON_* constants for bit-wise
      comparisons.
    
 * @param new_pressed_buttons new pressed_buttons value
 */
void
JoystickInterface::set_pressed_buttons(const uint32_t new_pressed_buttons)
{
  data->pressed_buttons = new_pressed_buttons;
  data_changed = true;
}

/** Get axis_x value.
 * X values of axes
 * @return axis_x value
 */
float *
JoystickInterface::axis_x() const
{
  return data->axis_x;
}

/** Get axis_x value at given index.
 * X values of axes
 * @param index index of value
 * @return axis_x value
 * @exception Exception thrown if index is out of bounds
 */
float
JoystickInterface::axis_x(unsigned int index) const
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  return data->axis_x[index];
}

/** Get maximum length of axis_x value.
 * @return length of axis_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::maxlenof_axis_x() const
{
  return 4;
}

/** Set axis_x value.
 * X values of axes
 * @param new_axis_x new axis_x value
 */
void
JoystickInterface::set_axis_x(const float * new_axis_x)
{
  memcpy(data->axis_x, new_axis_x, sizeof(float) * 4);
  data_changed = true;
}

/** Set axis_x value at given index.
 * X values of axes
 * @param new_axis_x new axis_x value
 * @param index index for of the value
 */
void
JoystickInterface::set_axis_x(unsigned int index, const float new_axis_x)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->axis_x[index] = new_axis_x;
}
/** Get axis_y value.
 * Y values of axes
 * @return axis_y value
 */
float *
JoystickInterface::axis_y() const
{
  return data->axis_y;
}

/** Get axis_y value at given index.
 * Y values of axes
 * @param index index of value
 * @return axis_y value
 * @exception Exception thrown if index is out of bounds
 */
float
JoystickInterface::axis_y(unsigned int index) const
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  return data->axis_y[index];
}

/** Get maximum length of axis_y value.
 * @return length of axis_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::maxlenof_axis_y() const
{
  return 4;
}

/** Set axis_y value.
 * Y values of axes
 * @param new_axis_y new axis_y value
 */
void
JoystickInterface::set_axis_y(const float * new_axis_y)
{
  memcpy(data->axis_y, new_axis_y, sizeof(float) * 4);
  data_changed = true;
}

/** Set axis_y value at given index.
 * Y values of axes
 * @param new_axis_y new axis_y value
 * @param index index for of the value
 */
void
JoystickInterface::set_axis_y(unsigned int index, const float new_axis_y)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->axis_y[index] = new_axis_y;
}
/* =========== message create =========== */
Message *
JoystickInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
JoystickInterface::copy_values(const Interface *other)
{
  const JoystickInterface *oi = dynamic_cast<const JoystickInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(JoystickInterface_data_t));
}

const char *
JoystickInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
JoystickInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(JoystickInterface)
/// @endcond


} // end namespace fawkes
