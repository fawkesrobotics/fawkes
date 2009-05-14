
/***************************************************************************
 *  JoystickInterface.cpp - Fawkes BlackBoard Interface - JoystickInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
 *
 *  $Id$
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
      Up to 32 buttons are support via an unsigned int bit field.
    
 * @ingroup FawkesInterfaces
 */


/** BUTTON_1 constant */
const unsigned int JoystickInterface::BUTTON_1 = 1;
/** BUTTON_2 constant */
const unsigned int JoystickInterface::BUTTON_2 = 2;
/** BUTTON_3 constant */
const unsigned int JoystickInterface::BUTTON_3 = 4;
/** BUTTON_4 constant */
const unsigned int JoystickInterface::BUTTON_4 = 8;
/** BUTTON_5 constant */
const unsigned int JoystickInterface::BUTTON_5 = 16;
/** BUTTON_6 constant */
const unsigned int JoystickInterface::BUTTON_6 = 32;
/** BUTTON_7 constant */
const unsigned int JoystickInterface::BUTTON_7 = 64;
/** BUTTON_8 constant */
const unsigned int JoystickInterface::BUTTON_8 = 128;
/** BUTTON_9 constant */
const unsigned int JoystickInterface::BUTTON_9 = 256;
/** BUTTON_10 constant */
const unsigned int JoystickInterface::BUTTON_10 = 512;
/** BUTTON_11 constant */
const unsigned int JoystickInterface::BUTTON_11 = 1024;
/** BUTTON_12 constant */
const unsigned int JoystickInterface::BUTTON_12 = 2048;
/** BUTTON_13 constant */
const unsigned int JoystickInterface::BUTTON_13 = 4096;
/** BUTTON_14 constant */
const unsigned int JoystickInterface::BUTTON_14 = 8192;
/** BUTTON_15 constant */
const unsigned int JoystickInterface::BUTTON_15 = 16384;
/** BUTTON_16 constant */
const unsigned int JoystickInterface::BUTTON_16 = 32768;
/** BUTTON_17 constant */
const unsigned int JoystickInterface::BUTTON_17 = 65536;
/** BUTTON_18 constant */
const unsigned int JoystickInterface::BUTTON_18 = 131072;
/** BUTTON_19 constant */
const unsigned int JoystickInterface::BUTTON_19 = 262144;
/** BUTTON_20 constant */
const unsigned int JoystickInterface::BUTTON_20 = 524288;
/** BUTTON_21 constant */
const unsigned int JoystickInterface::BUTTON_21 = 1048576;
/** BUTTON_22 constant */
const unsigned int JoystickInterface::BUTTON_22 = 2097152;
/** BUTTON_23 constant */
const unsigned int JoystickInterface::BUTTON_23 = 4194304;
/** BUTTON_24 constant */
const unsigned int JoystickInterface::BUTTON_24 = 8388608;
/** BUTTON_25 constant */
const unsigned int JoystickInterface::BUTTON_25 = 16777216;
/** BUTTON_26 constant */
const unsigned int JoystickInterface::BUTTON_26 = 33554432;
/** BUTTON_27 constant */
const unsigned int JoystickInterface::BUTTON_27 = 67108864;
/** BUTTON_28 constant */
const unsigned int JoystickInterface::BUTTON_28 = 134217728;
/** BUTTON_29 constant */
const unsigned int JoystickInterface::BUTTON_29 = 268435456;
/** BUTTON_30 constant */
const unsigned int JoystickInterface::BUTTON_30 = 536870912;
/** BUTTON_31 constant */
const unsigned int JoystickInterface::BUTTON_31 = 1073741824;
/** BUTTON_32 constant */
const unsigned int JoystickInterface::BUTTON_32 = 2147483648U;

/** Constructor */
JoystickInterface::JoystickInterface() : Interface()
{
  data_size = sizeof(JoystickInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (JoystickInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(Interface::IFT_STRING, "num_axes", 1, &data->num_axes);
  add_fieldinfo(Interface::IFT_STRING, "num_buttons", 1, &data->num_buttons);
  add_fieldinfo(Interface::IFT_UINT, "pressed_buttons", 1, &data->pressed_buttons);
  add_fieldinfo(Interface::IFT_FLOAT, "axis_x", 4, &data->axis_x);
  add_fieldinfo(Interface::IFT_FLOAT, "axis_y", 4, &data->axis_y);
  unsigned char tmp_hash[] = {0x5d, 0x1a, 0x93, 0x31, 0x57, 0x5d, 0x6c, 0x7, 0x40, 0xb5, 0xcd, 0x4c, 0xba, 0x8b, 0x82, 0xa1};
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
char
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
JoystickInterface::set_num_axes(const char new_num_axes)
{
  data->num_axes = new_num_axes;
}

/** Get num_buttons value.
 * 
      The number of buttons of this joystick.
    
 * @return num_buttons value
 */
char
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
JoystickInterface::set_num_buttons(const char new_num_buttons)
{
  data->num_buttons = new_num_buttons;
}

/** Get pressed_buttons value.
 * 
      A bit field of enabled buttons. For each currently clicked button the
      corresponding bit is set to 1. Use the BUTTON_* constants for bit-wise
      comparisons.
    
 * @return pressed_buttons value
 */
unsigned int
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
JoystickInterface::set_pressed_buttons(const unsigned int new_pressed_buttons)
{
  data->pressed_buttons = new_pressed_buttons;
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
