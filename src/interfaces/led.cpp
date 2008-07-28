
/***************************************************************************
 *  led.cpp - Fawkes BlackBoard Interface - LedInterface
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

#include <interfaces/led.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class LedInterface <interfaces/led.h>
 * LedInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to LEDs. The interface controls an
      intensity value between 0.0 (off) and 1.0 (on, max intensity). LEDs
      that do not support intensity setting can only be set to on and off.
    
 */


/** ON constant */
const float LedInterface::ON = 1.0;
/** OFF constant */
const float LedInterface::OFF = 0.0;

/** Constructor */
LedInterface::LedInterface() : Interface()
{
  data_size = sizeof(LedInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (LedInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(Interface::IFT_FLOAT, "intensity", &data->intensity);
  unsigned char tmp_hash[] = {0xae, 0xbe, 0xe2, 0xba, 0xf7, 0x15, 0xbd, 0xd3, 0x2, 0x74, 0x4a, 0x6a, 0xe9, 0xbf, 0x75, 0xab};
  set_hash(tmp_hash);
}

/** Destructor */
LedInterface::~LedInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get intensity value.
 * Intensity value.
 * @return intensity value
 */
float
LedInterface::intensity() const
{
  return data->intensity;
}

/** Get maximum length of intensity value.
 * @return length of intensity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LedInterface::maxlenof_intensity() const
{
  return 1;
}

/** Set intensity value.
 * Intensity value.
 * @param new_intensity new intensity value
 */
void
LedInterface::set_intensity(const float new_intensity)
{
  data->intensity = new_intensity;
}

/* =========== message create =========== */
Message *
LedInterface::create_message(const char *type) const
{
  if ( strncmp("SetIntensityMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetIntensityMessage();
  } else if ( strncmp("TurnOnMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new TurnOnMessage();
  } else if ( strncmp("TurnOffMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new TurnOffMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/* =========== messages =========== */
/** @class LedInterface::SetIntensityMessage interfaces/led.h
 * SetIntensityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_intensity initial value for intensity
 */
LedInterface::SetIntensityMessage::SetIntensityMessage(const float ini_intensity) : Message("SetIntensityMessage")
{
  data_size = sizeof(SetIntensityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetIntensityMessage_data_t *)data_ptr;
  data->intensity = ini_intensity;
}
/** Constructor */
LedInterface::SetIntensityMessage::SetIntensityMessage() : Message("SetIntensityMessage")
{
  data_size = sizeof(SetIntensityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetIntensityMessage_data_t *)data_ptr;
}

/** Destructor */
LedInterface::SetIntensityMessage::~SetIntensityMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
LedInterface::SetIntensityMessage::SetIntensityMessage(const SetIntensityMessage *m) : Message("SetIntensityMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetIntensityMessage_data_t *)data_ptr;
}

/* Methods */
/** Get intensity value.
 * Intensity value.
 * @return intensity value
 */
float
LedInterface::SetIntensityMessage::intensity() const
{
  return data->intensity;
}

/** Get maximum length of intensity value.
 * @return length of intensity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LedInterface::SetIntensityMessage::maxlenof_intensity() const
{
  return 1;
}

/** Set intensity value.
 * Intensity value.
 * @param new_intensity new intensity value
 */
void
LedInterface::SetIntensityMessage::set_intensity(const float new_intensity)
{
  data->intensity = new_intensity;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
LedInterface::SetIntensityMessage::clone() const
{
  return new LedInterface::SetIntensityMessage(this);
}
/** @class LedInterface::TurnOnMessage interfaces/led.h
 * TurnOnMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
LedInterface::TurnOnMessage::TurnOnMessage() : Message("TurnOnMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
LedInterface::TurnOnMessage::~TurnOnMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
LedInterface::TurnOnMessage::TurnOnMessage(const TurnOnMessage *m) : Message("TurnOnMessage")
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
LedInterface::TurnOnMessage::clone() const
{
  return new LedInterface::TurnOnMessage(this);
}
/** @class LedInterface::TurnOffMessage interfaces/led.h
 * TurnOffMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
LedInterface::TurnOffMessage::TurnOffMessage() : Message("TurnOffMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
LedInterface::TurnOffMessage::~TurnOffMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
LedInterface::TurnOffMessage::TurnOffMessage(const TurnOffMessage *m) : Message("TurnOffMessage")
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
LedInterface::TurnOffMessage::clone() const
{
  return new LedInterface::TurnOffMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
LedInterface::message_valid(const Message *message) const
{
  const SetIntensityMessage *m0 = dynamic_cast<const SetIntensityMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const TurnOnMessage *m1 = dynamic_cast<const TurnOnMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const TurnOffMessage *m2 = dynamic_cast<const TurnOffMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(LedInterface)
/// @endcond


} // end namespace fawkes
