
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

#include <map>
#include <string>
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
/** JFF_RUMBLE constant */
const uint8_t JoystickInterface::JFF_RUMBLE = 1;
/** JFF_PERIODIC constant */
const uint8_t JoystickInterface::JFF_PERIODIC = 2;
/** JFF_RAMP constant */
const uint8_t JoystickInterface::JFF_RAMP = 4;
/** JFF_SPRING constant */
const uint8_t JoystickInterface::JFF_SPRING = 8;
/** JFF_FRICTION constant */
const uint8_t JoystickInterface::JFF_FRICTION = 16;
/** JFF_DAMPER constant */
const uint8_t JoystickInterface::JFF_DAMPER = 32;
/** JFF_INERTIA constant */
const uint8_t JoystickInterface::JFF_INERTIA = 64;
/** JFF_CONSTANT constant */
const uint8_t JoystickInterface::JFF_CONSTANT = 128;

/** Constructor */
JoystickInterface::JoystickInterface() : Interface()
{
  data_size = sizeof(JoystickInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (JoystickInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_Direction[(int)DIRECTION_DOWN] = "DIRECTION_DOWN";
  enum_map_Direction[(int)DIRECTION_LEFT] = "DIRECTION_LEFT";
  enum_map_Direction[(int)DIRECTION_UP] = "DIRECTION_UP";
  enum_map_Direction[(int)DIRECTION_RIGHT] = "DIRECTION_RIGHT";
  add_fieldinfo(IFT_BYTE, "num_axes", 1, &data->num_axes);
  add_fieldinfo(IFT_BYTE, "num_buttons", 1, &data->num_buttons);
  add_fieldinfo(IFT_BYTE, "supported_ff_effects", 1, &data->supported_ff_effects);
  add_fieldinfo(IFT_UINT32, "pressed_buttons", 1, &data->pressed_buttons);
  add_fieldinfo(IFT_FLOAT, "axis", 8, &data->axis);
  add_fieldinfo(IFT_UINT8, "ff_effects", 1, &data->ff_effects);
  add_messageinfo("StartRumbleMessage");
  add_messageinfo("StopRumbleMessage");
  add_messageinfo("StopAllMessage");
  unsigned char tmp_hash[] = {0xeb, 0x7c, 0xd1, 0x1c, 0xae, 0xa, 0x37, 0x45, 0x5c, 0xa, 0x5e, 0xda, 0x5e, 0x17, 0xdd, 0x42};
  set_hash(tmp_hash);
}

/** Destructor */
JoystickInterface::~JoystickInterface()
{
  free(data_ptr);
}
/** Convert Direction constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
JoystickInterface::tostring_Direction(Direction value) const
{
  switch (value) {
  case DIRECTION_DOWN: return "DIRECTION_DOWN";
  case DIRECTION_LEFT: return "DIRECTION_LEFT";
  case DIRECTION_UP: return "DIRECTION_UP";
  case DIRECTION_RIGHT: return "DIRECTION_RIGHT";
  default: return "UNKNOWN";
  }
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

/** Get supported_ff_effects value.
 * 
      Bit field indicating available force-feedback effects.
    
 * @return supported_ff_effects value
 */
uint8_t
JoystickInterface::supported_ff_effects() const
{
  return data->supported_ff_effects;
}

/** Get maximum length of supported_ff_effects value.
 * @return length of supported_ff_effects value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::maxlenof_supported_ff_effects() const
{
  return 1;
}

/** Set supported_ff_effects value.
 * 
      Bit field indicating available force-feedback effects.
    
 * @param new_supported_ff_effects new supported_ff_effects value
 */
void
JoystickInterface::set_supported_ff_effects(const uint8_t new_supported_ff_effects)
{
  data->supported_ff_effects = new_supported_ff_effects;
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

/** Get axis value.
 * Values of axes.
 * @return axis value
 */
float *
JoystickInterface::axis() const
{
  return data->axis;
}

/** Get axis value at given index.
 * Values of axes.
 * @param index index of value
 * @return axis value
 * @exception Exception thrown if index is out of bounds
 */
float
JoystickInterface::axis(unsigned int index) const
{
  if (index > 8) {
    throw Exception("Index value %u out of bounds (0..8)", index);
  }
  return data->axis[index];
}

/** Get maximum length of axis value.
 * @return length of axis value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::maxlenof_axis() const
{
  return 8;
}

/** Set axis value.
 * Values of axes.
 * @param new_axis new axis value
 */
void
JoystickInterface::set_axis(const float * new_axis)
{
  memcpy(data->axis, new_axis, sizeof(float) * 8);
  data_changed = true;
}

/** Set axis value at given index.
 * Values of axes.
 * @param new_axis new axis value
 * @param index index for of the value
 */
void
JoystickInterface::set_axis(unsigned int index, const float new_axis)
{
  if (index > 8) {
    throw Exception("Index value %u out of bounds (0..8)", index);
  }
  data->axis[index] = new_axis;
  data_changed = true;
}
/** Get ff_effects value.
 * 
      Currently running effects. Either 0 if no effect is running, or a bit-wise
      ored field of the JFF constants.
    
 * @return ff_effects value
 */
uint8_t
JoystickInterface::ff_effects() const
{
  return data->ff_effects;
}

/** Get maximum length of ff_effects value.
 * @return length of ff_effects value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::maxlenof_ff_effects() const
{
  return 1;
}

/** Set ff_effects value.
 * 
      Currently running effects. Either 0 if no effect is running, or a bit-wise
      ored field of the JFF constants.
    
 * @param new_ff_effects new ff_effects value
 */
void
JoystickInterface::set_ff_effects(const uint8_t new_ff_effects)
{
  data->ff_effects = new_ff_effects;
  data_changed = true;
}

/* =========== message create =========== */
Message *
JoystickInterface::create_message(const char *type) const
{
  if ( strncmp("StartRumbleMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StartRumbleMessage();
  } else if ( strncmp("StopRumbleMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopRumbleMessage();
  } else if ( strncmp("StopAllMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopAllMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
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
  if (strcmp(enumtype, "Direction") == 0) {
    return tostring_Direction((Direction)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class JoystickInterface::StartRumbleMessage <interfaces/JoystickInterface.h>
 * StartRumbleMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_length initial value for length
 * @param ini_delay initial value for delay
 * @param ini_direction initial value for direction
 * @param ini_strong_magnitude initial value for strong_magnitude
 * @param ini_weak_magnitude initial value for weak_magnitude
 */
JoystickInterface::StartRumbleMessage::StartRumbleMessage(const uint16_t ini_length, const uint16_t ini_delay, const Direction ini_direction, const uint16_t ini_strong_magnitude, const uint16_t ini_weak_magnitude) : Message("StartRumbleMessage")
{
  data_size = sizeof(StartRumbleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StartRumbleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->length = ini_length;
  data->delay = ini_delay;
  data->direction = ini_direction;
  data->strong_magnitude = ini_strong_magnitude;
  data->weak_magnitude = ini_weak_magnitude;
  enum_map_Direction[(int)DIRECTION_DOWN] = "DIRECTION_DOWN";
  enum_map_Direction[(int)DIRECTION_LEFT] = "DIRECTION_LEFT";
  enum_map_Direction[(int)DIRECTION_UP] = "DIRECTION_UP";
  enum_map_Direction[(int)DIRECTION_RIGHT] = "DIRECTION_RIGHT";
  add_fieldinfo(IFT_UINT16, "length", 1, &data->length);
  add_fieldinfo(IFT_UINT16, "delay", 1, &data->delay);
  add_fieldinfo(IFT_ENUM, "direction", 1, &data->direction, "Direction", &enum_map_Direction);
  add_fieldinfo(IFT_UINT16, "strong_magnitude", 1, &data->strong_magnitude);
  add_fieldinfo(IFT_UINT16, "weak_magnitude", 1, &data->weak_magnitude);
}
/** Constructor */
JoystickInterface::StartRumbleMessage::StartRumbleMessage() : Message("StartRumbleMessage")
{
  data_size = sizeof(StartRumbleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StartRumbleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Direction[(int)DIRECTION_DOWN] = "DIRECTION_DOWN";
  enum_map_Direction[(int)DIRECTION_LEFT] = "DIRECTION_LEFT";
  enum_map_Direction[(int)DIRECTION_UP] = "DIRECTION_UP";
  enum_map_Direction[(int)DIRECTION_RIGHT] = "DIRECTION_RIGHT";
  add_fieldinfo(IFT_UINT16, "length", 1, &data->length);
  add_fieldinfo(IFT_UINT16, "delay", 1, &data->delay);
  add_fieldinfo(IFT_ENUM, "direction", 1, &data->direction, "Direction", &enum_map_Direction);
  add_fieldinfo(IFT_UINT16, "strong_magnitude", 1, &data->strong_magnitude);
  add_fieldinfo(IFT_UINT16, "weak_magnitude", 1, &data->weak_magnitude);
}

/** Destructor */
JoystickInterface::StartRumbleMessage::~StartRumbleMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JoystickInterface::StartRumbleMessage::StartRumbleMessage(const StartRumbleMessage *m) : Message("StartRumbleMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StartRumbleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get length value.
 * Effect length in ms.
       Setting to 0 will make the effect to play continuously until stopped.
    
 * @return length value
 */
uint16_t
JoystickInterface::StartRumbleMessage::length() const
{
  return data->length;
}

/** Get maximum length of length value.
 * @return length of length value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::StartRumbleMessage::maxlenof_length() const
{
  return 1;
}

/** Set length value.
 * Effect length in ms.
       Setting to 0 will make the effect to play continuously until stopped.
    
 * @param new_length new length value
 */
void
JoystickInterface::StartRumbleMessage::set_length(const uint16_t new_length)
{
  data->length = new_length;
}

/** Get delay value.
 * Delay before effect starts in ms.
 * @return delay value
 */
uint16_t
JoystickInterface::StartRumbleMessage::delay() const
{
  return data->delay;
}

/** Get maximum length of delay value.
 * @return length of delay value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::StartRumbleMessage::maxlenof_delay() const
{
  return 1;
}

/** Set delay value.
 * Delay before effect starts in ms.
 * @param new_delay new delay value
 */
void
JoystickInterface::StartRumbleMessage::set_delay(const uint16_t new_delay)
{
  data->delay = new_delay;
}

/** Get direction value.
 * Direction of effect
 * @return direction value
 */
JoystickInterface::Direction
JoystickInterface::StartRumbleMessage::direction() const
{
  return (JoystickInterface::Direction)data->direction;
}

/** Get maximum length of direction value.
 * @return length of direction value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::StartRumbleMessage::maxlenof_direction() const
{
  return 1;
}

/** Set direction value.
 * Direction of effect
 * @param new_direction new direction value
 */
void
JoystickInterface::StartRumbleMessage::set_direction(const Direction new_direction)
{
  data->direction = new_direction;
}

/** Get strong_magnitude value.
 * Magnitude of heavy motor.
 * @return strong_magnitude value
 */
uint16_t
JoystickInterface::StartRumbleMessage::strong_magnitude() const
{
  return data->strong_magnitude;
}

/** Get maximum length of strong_magnitude value.
 * @return length of strong_magnitude value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::StartRumbleMessage::maxlenof_strong_magnitude() const
{
  return 1;
}

/** Set strong_magnitude value.
 * Magnitude of heavy motor.
 * @param new_strong_magnitude new strong_magnitude value
 */
void
JoystickInterface::StartRumbleMessage::set_strong_magnitude(const uint16_t new_strong_magnitude)
{
  data->strong_magnitude = new_strong_magnitude;
}

/** Get weak_magnitude value.
 * Magnitude of light motor.
 * @return weak_magnitude value
 */
uint16_t
JoystickInterface::StartRumbleMessage::weak_magnitude() const
{
  return data->weak_magnitude;
}

/** Get maximum length of weak_magnitude value.
 * @return length of weak_magnitude value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JoystickInterface::StartRumbleMessage::maxlenof_weak_magnitude() const
{
  return 1;
}

/** Set weak_magnitude value.
 * Magnitude of light motor.
 * @param new_weak_magnitude new weak_magnitude value
 */
void
JoystickInterface::StartRumbleMessage::set_weak_magnitude(const uint16_t new_weak_magnitude)
{
  data->weak_magnitude = new_weak_magnitude;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
JoystickInterface::StartRumbleMessage::clone() const
{
  return new JoystickInterface::StartRumbleMessage(this);
}
/** @class JoystickInterface::StopRumbleMessage <interfaces/JoystickInterface.h>
 * StopRumbleMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
JoystickInterface::StopRumbleMessage::StopRumbleMessage() : Message("StopRumbleMessage")
{
  data_size = sizeof(StopRumbleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopRumbleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Direction[(int)DIRECTION_DOWN] = "DIRECTION_DOWN";
  enum_map_Direction[(int)DIRECTION_LEFT] = "DIRECTION_LEFT";
  enum_map_Direction[(int)DIRECTION_UP] = "DIRECTION_UP";
  enum_map_Direction[(int)DIRECTION_RIGHT] = "DIRECTION_RIGHT";
}

/** Destructor */
JoystickInterface::StopRumbleMessage::~StopRumbleMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JoystickInterface::StopRumbleMessage::StopRumbleMessage(const StopRumbleMessage *m) : Message("StopRumbleMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StopRumbleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
JoystickInterface::StopRumbleMessage::clone() const
{
  return new JoystickInterface::StopRumbleMessage(this);
}
/** @class JoystickInterface::StopAllMessage <interfaces/JoystickInterface.h>
 * StopAllMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
JoystickInterface::StopAllMessage::StopAllMessage() : Message("StopAllMessage")
{
  data_size = sizeof(StopAllMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopAllMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Direction[(int)DIRECTION_DOWN] = "DIRECTION_DOWN";
  enum_map_Direction[(int)DIRECTION_LEFT] = "DIRECTION_LEFT";
  enum_map_Direction[(int)DIRECTION_UP] = "DIRECTION_UP";
  enum_map_Direction[(int)DIRECTION_RIGHT] = "DIRECTION_RIGHT";
}

/** Destructor */
JoystickInterface::StopAllMessage::~StopAllMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JoystickInterface::StopAllMessage::StopAllMessage(const StopAllMessage *m) : Message("StopAllMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StopAllMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
JoystickInterface::StopAllMessage::clone() const
{
  return new JoystickInterface::StopAllMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
JoystickInterface::message_valid(const Message *message) const
{
  const StartRumbleMessage *m0 = dynamic_cast<const StartRumbleMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const StopRumbleMessage *m1 = dynamic_cast<const StopRumbleMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const StopAllMessage *m2 = dynamic_cast<const StopAllMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(JoystickInterface)
/// @endcond


} // end namespace fawkes
