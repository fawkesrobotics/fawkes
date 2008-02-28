
/***************************************************************************
 *  battery.cpp - Fawkes BlackBoard Interface - BatteryInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
 */

#include <interfaces/battery.h>

#include <cstring>
#include <cstdlib>

/** @class BatteryInterface interfaces/battery.h
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
  unsigned char tmp_hash[] = {0x12, 0x80, 0x3d, 0x25, 0xc5, 0xb4, 0x8e, 0x69, 00, 0xe6, 0xcc, 0xc0, 0x68, 0x17, 0x2f, 0x4};
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

/** Set temperature value.
 * Battery Temperature [°C]
 * @param new_temperature new temperature value
 */
void
BatteryInterface::set_temperature(const unsigned int new_temperature)
{
  data->temperature = new_temperature;
}

/* =========== messages =========== */
/** @class BatteryInterface::push_buttonMessage interfaces/battery.h
 * push_buttonMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
BatteryInterface::push_buttonMessage::push_buttonMessage() : Message("push_buttonMessage")
{
  data_size = sizeof(push_buttonMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (push_buttonMessage_data_t *)data_ptr;
}

/** Destructor */
BatteryInterface::push_buttonMessage::~push_buttonMessage()
{
  free(data_ptr);
}

/* Methods */
/** @class BatteryInterface::sleepMessage interfaces/battery.h
 * sleepMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
BatteryInterface::sleepMessage::sleepMessage() : Message("sleepMessage")
{
  data_size = sizeof(sleepMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (sleepMessage_data_t *)data_ptr;
}

/** Destructor */
BatteryInterface::sleepMessage::~sleepMessage()
{
  free(data_ptr);
}

/* Methods */
/** Check if message is valid an can be queued.
 * @param message Message to check
 */
bool
BatteryInterface::message_valid(const Message *message) const
{
  const push_buttonMessage *m0 = dynamic_cast<const push_buttonMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const sleepMessage *m1 = dynamic_cast<const sleepMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(BatteryInterface)
/// @endcond

