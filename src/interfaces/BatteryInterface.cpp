
/***************************************************************************
 *  BatteryInterface.cpp - Fawkes BlackBoard Interface - BatteryInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Daniel Beck
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

#include <interfaces/BatteryInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class BatteryInterface <interfaces/BatteryInterface.h>
 * BatteryInterface Fawkes BlackBoard Interface.
 * This interface contains status information about the
    battery. In addition to this it allows to send messages which
    turn the battery on/off
 * @ingroup FawkesInterfaces
 */



/** Constructor */
BatteryInterface::BatteryInterface() : Interface()
{
  data_size = sizeof(BatteryInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (BatteryInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "current", 1, &data->current);
  add_fieldinfo(IFT_UINT32, "voltage", 1, &data->voltage);
  add_fieldinfo(IFT_UINT32, "temperature", 1, &data->temperature);
  add_fieldinfo(IFT_FLOAT, "absolute_soc", 1, &data->absolute_soc);
  add_fieldinfo(IFT_FLOAT, "relative_soc", 1, &data->relative_soc);
  add_messageinfo("PushButtonMessage");
  add_messageinfo("SleepMessage");
  unsigned char tmp_hash[] = {0x28, 0xb6, 0xbe, 0xe7, 0xf1, 0x47, 0x2, 0x12, 0x1d, 0xe3, 0x7c, 0x14, 0xe9, 0x1f, 0x24, 0x4d};
  set_hash(tmp_hash);
}

/** Destructor */
BatteryInterface::~BatteryInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get current value.
 * Battery Current [mA]
 * @return current value
 */
uint32_t
BatteryInterface::current() const
{
  return data->current;
}

/** Get maximum length of current value.
 * @return length of current value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
BatteryInterface::maxlenof_current() const
{
  return 1;
}

/** Set current value.
 * Battery Current [mA]
 * @param new_current new current value
 */
void
BatteryInterface::set_current(const uint32_t new_current)
{
  data->current = new_current;
  data_changed = true;
}

/** Get voltage value.
 * Battery Voltage [mV]
 * @return voltage value
 */
uint32_t
BatteryInterface::voltage() const
{
  return data->voltage;
}

/** Get maximum length of voltage value.
 * @return length of voltage value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
BatteryInterface::maxlenof_voltage() const
{
  return 1;
}

/** Set voltage value.
 * Battery Voltage [mV]
 * @param new_voltage new voltage value
 */
void
BatteryInterface::set_voltage(const uint32_t new_voltage)
{
  data->voltage = new_voltage;
  data_changed = true;
}

/** Get temperature value.
 * Battery Temperature [°C]
 * @return temperature value
 */
uint32_t
BatteryInterface::temperature() const
{
  return data->temperature;
}

/** Get maximum length of temperature value.
 * @return length of temperature value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
BatteryInterface::maxlenof_temperature() const
{
  return 1;
}

/** Set temperature value.
 * Battery Temperature [°C]
 * @param new_temperature new temperature value
 */
void
BatteryInterface::set_temperature(const uint32_t new_temperature)
{
  data->temperature = new_temperature;
  data_changed = true;
}

/** Get absolute_soc value.
 * Absolute state of charge [%]
 * @return absolute_soc value
 */
float
BatteryInterface::absolute_soc() const
{
  return data->absolute_soc;
}

/** Get maximum length of absolute_soc value.
 * @return length of absolute_soc value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
BatteryInterface::maxlenof_absolute_soc() const
{
  return 1;
}

/** Set absolute_soc value.
 * Absolute state of charge [%]
 * @param new_absolute_soc new absolute_soc value
 */
void
BatteryInterface::set_absolute_soc(const float new_absolute_soc)
{
  data->absolute_soc = new_absolute_soc;
  data_changed = true;
}

/** Get relative_soc value.
 * Relative state of charge [%]
 * @return relative_soc value
 */
float
BatteryInterface::relative_soc() const
{
  return data->relative_soc;
}

/** Get maximum length of relative_soc value.
 * @return length of relative_soc value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
BatteryInterface::maxlenof_relative_soc() const
{
  return 1;
}

/** Set relative_soc value.
 * Relative state of charge [%]
 * @param new_relative_soc new relative_soc value
 */
void
BatteryInterface::set_relative_soc(const float new_relative_soc)
{
  data->relative_soc = new_relative_soc;
  data_changed = true;
}

/* =========== message create =========== */
Message *
BatteryInterface::create_message(const char *type) const
{
  if ( strncmp("PushButtonMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new PushButtonMessage();
  } else if ( strncmp("SleepMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SleepMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
BatteryInterface::copy_values(const Interface *other)
{
  const BatteryInterface *oi = dynamic_cast<const BatteryInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(BatteryInterface_data_t));
}

const char *
BatteryInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class BatteryInterface::PushButtonMessage <interfaces/BatteryInterface.h>
 * PushButtonMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
BatteryInterface::PushButtonMessage::PushButtonMessage() : Message("PushButtonMessage")
{
  data_size = sizeof(PushButtonMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PushButtonMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
BatteryInterface::PushButtonMessage::~PushButtonMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
BatteryInterface::PushButtonMessage::PushButtonMessage(const PushButtonMessage *m) : Message("PushButtonMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (PushButtonMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
BatteryInterface::PushButtonMessage::clone() const
{
  return new BatteryInterface::PushButtonMessage(this);
}
/** @class BatteryInterface::SleepMessage <interfaces/BatteryInterface.h>
 * SleepMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
BatteryInterface::SleepMessage::SleepMessage() : Message("SleepMessage")
{
  data_size = sizeof(SleepMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SleepMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
BatteryInterface::SleepMessage::~SleepMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
BatteryInterface::SleepMessage::SleepMessage(const SleepMessage *m) : Message("SleepMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SleepMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
BatteryInterface::SleepMessage::clone() const
{
  return new BatteryInterface::SleepMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
BatteryInterface::message_valid(const Message *message) const
{
  const PushButtonMessage *m0 = dynamic_cast<const PushButtonMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SleepMessage *m1 = dynamic_cast<const SleepMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(BatteryInterface)
/// @endcond


} // end namespace fawkes
