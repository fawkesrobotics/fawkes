
/***************************************************************************
 *  battery.cpp - Fawkes BlackBoard Interface - BatteryInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Daniel Beck
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

#include <interfaces/battery.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class BatteryInterface <interfaces/battery.h>
 * BatteryInterface Fawkes BlackBoard Interface.
 * This interface contains status information about the
    battery. In addition to this it allows to send messages which
    turn the battery on/off
 */



/** Constructor */
BatteryInterface::BatteryInterface() : Interface()
{
  data_size = sizeof(BatteryInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (BatteryInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(Interface::IFT_UINT, "current", &data->current);
  add_fieldinfo(Interface::IFT_UINT, "voltage", &data->voltage);
  add_fieldinfo(Interface::IFT_UINT, "temperature", &data->temperature);
  unsigned char tmp_hash[] = {0x1b, 0x21, 0x6b, 0x87, 0xfd, 0x51, 0x4c, 0xf1, 0xec, 0x82, 0xb4, 0xa3, 0x6f, 0xbf, 0x7e, 0x25};
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
unsigned int
BatteryInterface::current()
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
BatteryInterface::set_current(const unsigned int new_current)
{
  data->current = new_current;
}

/** Get voltage value.
 * Battery Voltage [mV]
 * @return voltage value
 */
unsigned int
BatteryInterface::voltage()
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
BatteryInterface::set_voltage(const unsigned int new_voltage)
{
  data->voltage = new_voltage;
}

/** Get temperature value.
 * Battery Temperature [°C]
 * @return temperature value
 */
unsigned int
BatteryInterface::temperature()
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
BatteryInterface::set_temperature(const unsigned int new_temperature)
{
  data->temperature = new_temperature;
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


/* =========== messages =========== */
/** @class BatteryInterface::PushButtonMessage interfaces/battery.h
 * PushButtonMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
BatteryInterface::PushButtonMessage::PushButtonMessage() : Message("PushButtonMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
BatteryInterface::PushButtonMessage::~PushButtonMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
BatteryInterface::PushButtonMessage::PushButtonMessage(const PushButtonMessage *m) : Message("PushButtonMessage")
{
  data_size = 0;
  data_ptr  = NULL;
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
/** @class BatteryInterface::SleepMessage interfaces/battery.h
 * SleepMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
BatteryInterface::SleepMessage::SleepMessage() : Message("SleepMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
BatteryInterface::SleepMessage::~SleepMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
BatteryInterface::SleepMessage::SleepMessage(const SleepMessage *m) : Message("SleepMessage")
{
  data_size = 0;
  data_ptr  = NULL;
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
