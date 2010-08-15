
/***************************************************************************
 *  LedInterface.cpp - Fawkes BlackBoard Interface - LedInterface
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

#include <interfaces/LedInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class LedInterface <interfaces/LedInterface.h>
 * LedInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to LEDs. The interface controls an
      intensity value between 0.0 (off) and 1.0 (on, max intensity). LEDs
      that do not support intensity setting can only be set to on and off.
    
 * @ingroup FawkesInterfaces
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
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_FLOAT, "intensity", 1, &data->intensity);
  add_messageinfo("SetIntensityMessage");
  add_messageinfo("TurnOnMessage");
  add_messageinfo("TurnOffMessage");
  unsigned char tmp_hash[] = {0xd, 0x86, 0x60, 0xcd, 0xae, 0x41, 0xa5, 0xa1, 0xbc, 0xb7, 0xf, 0x9, 0x90, 00, 0x4d, 0x40};
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
  data_changed = true;
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


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
LedInterface::copy_values(const Interface *other)
{
  const LedInterface *oi = dynamic_cast<const LedInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(LedInterface_data_t));
}

const char *
LedInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class LedInterface::SetIntensityMessage <interfaces/LedInterface.h>
 * SetIntensityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_time_sec initial value for time_sec
 * @param ini_intensity initial value for intensity
 */
LedInterface::SetIntensityMessage::SetIntensityMessage(const float ini_time_sec, const float ini_intensity) : Message("SetIntensityMessage")
{
  data_size = sizeof(SetIntensityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetIntensityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->time_sec = ini_time_sec;
  data->intensity = ini_intensity;
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_FLOAT, "intensity", 1, &data->intensity);
}
/** Constructor */
LedInterface::SetIntensityMessage::SetIntensityMessage() : Message("SetIntensityMessage")
{
  data_size = sizeof(SetIntensityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetIntensityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_FLOAT, "intensity", 1, &data->intensity);
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
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get time_sec value.
 * 
      Time in seconds when to reach the intensity.
    
 * @return time_sec value
 */
float
LedInterface::SetIntensityMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LedInterface::SetIntensityMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * 
      Time in seconds when to reach the intensity.
    
 * @param new_time_sec new time_sec value
 */
void
LedInterface::SetIntensityMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

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
/** @class LedInterface::TurnOnMessage <interfaces/LedInterface.h>
 * TurnOnMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
LedInterface::TurnOnMessage::TurnOnMessage() : Message("TurnOnMessage")
{
  data_size = sizeof(TurnOnMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TurnOnMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
LedInterface::TurnOnMessage::~TurnOnMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
LedInterface::TurnOnMessage::TurnOnMessage(const TurnOnMessage *m) : Message("TurnOnMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (TurnOnMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
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
/** @class LedInterface::TurnOffMessage <interfaces/LedInterface.h>
 * TurnOffMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
LedInterface::TurnOffMessage::TurnOffMessage() : Message("TurnOffMessage")
{
  data_size = sizeof(TurnOffMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TurnOffMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
LedInterface::TurnOffMessage::~TurnOffMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
LedInterface::TurnOffMessage::TurnOffMessage(const TurnOffMessage *m) : Message("TurnOffMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (TurnOffMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
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
