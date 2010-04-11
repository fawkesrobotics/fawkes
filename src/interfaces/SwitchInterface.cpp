
/***************************************************************************
 *  SwitchInterface.cpp - Fawkes BlackBoard Interface - SwitchInterface
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

#include <interfaces/SwitchInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class SwitchInterface <interfaces/SwitchInterface.h>
 * SwitchInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to LEDs. The interface controls
      an intensity value between 0.0 (off) and 1.0 (on, max
      intensity). LEDs that do not support intensity setting can only
      be set to on and off.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
SwitchInterface::SwitchInterface() : Interface()
{
  data_size = sizeof(SwitchInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (SwitchInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_FLOAT, "history", 1, &data->history);
  add_fieldinfo(IFT_UINT, "short_activations", 1, &data->short_activations);
  add_fieldinfo(IFT_UINT, "long_activations", 1, &data->long_activations);
  add_fieldinfo(IFT_UINT, "activation_count", 1, &data->activation_count);
  add_messageinfo("SetMessage");
  add_messageinfo("EnableSwitchMessage");
  add_messageinfo("DisableSwitchMessage");
  add_messageinfo("EnableDurationMessage");
  unsigned char tmp_hash[] = {0x23, 0x86, 0x5, 0xe0, 0x29, 0xf5, 0x17, 0x7e, 0x1e, 0x4b, 0xf1, 0xdf, 0xd9, 0xa7, 0xbe, 0x31};
  set_hash(tmp_hash);
}

/** Destructor */
SwitchInterface::~SwitchInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get enabled value.
 * 
      True if the switch is currently enabled.
    
 * @return enabled value
 */
bool
SwitchInterface::is_enabled() const
{
  return data->enabled;
}

/** Get maximum length of enabled value.
 * @return length of enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SwitchInterface::maxlenof_enabled() const
{
  return 1;
}

/** Set enabled value.
 * 
      True if the switch is currently enabled.
    
 * @param new_enabled new enabled value
 */
void
SwitchInterface::set_enabled(const bool new_enabled)
{
  data->enabled = new_enabled;
}

/** Get value value.
 * 
      If switches support multiple states these can be indicated with
      this value. For example for a switch that notes the intensity it
      could be a value in the valid range.
    
 * @return value value
 */
float
SwitchInterface::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SwitchInterface::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * 
      If switches support multiple states these can be indicated with
      this value. For example for a switch that notes the intensity it
      could be a value in the valid range.
    
 * @param new_value new value value
 */
void
SwitchInterface::set_value(const float new_value)
{
  data->value = new_value;
}

/** Get history value.
 * 
      This value records the number of seconds a switch has been
      enabled continuously -- or not. The time is recorded in
      seconds. A positive value indicates time the switch was turned
      on, a negative value indicates the time (when converted to the
      absolute value) the button has not been pressed. Zero means
      "just initialized".
    
 * @return history value
 */
float
SwitchInterface::history() const
{
  return data->history;
}

/** Get maximum length of history value.
 * @return length of history value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SwitchInterface::maxlenof_history() const
{
  return 1;
}

/** Set history value.
 * 
      This value records the number of seconds a switch has been
      enabled continuously -- or not. The time is recorded in
      seconds. A positive value indicates time the switch was turned
      on, a negative value indicates the time (when converted to the
      absolute value) the button has not been pressed. Zero means
      "just initialized".
    
 * @param new_history new history value
 */
void
SwitchInterface::set_history(const float new_history)
{
  data->history = new_history;
}

/** Get short_activations value.
 * 
      Number of consecutive short clicks (turned on). Can be used to recognize
      patterns of clicks. This is an optional field.
    
 * @return short_activations value
 */
unsigned int
SwitchInterface::short_activations() const
{
  return data->short_activations;
}

/** Get maximum length of short_activations value.
 * @return length of short_activations value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SwitchInterface::maxlenof_short_activations() const
{
  return 1;
}

/** Set short_activations value.
 * 
      Number of consecutive short clicks (turned on). Can be used to recognize
      patterns of clicks. This is an optional field.
    
 * @param new_short_activations new short_activations value
 */
void
SwitchInterface::set_short_activations(const unsigned int new_short_activations)
{
  data->short_activations = new_short_activations;
}

/** Get long_activations value.
 * 
      Number of consecutive short clicks (turned on). Can be used to recognize
      patterns of clicks. This is an optional field.
    
 * @return long_activations value
 */
unsigned int
SwitchInterface::long_activations() const
{
  return data->long_activations;
}

/** Get maximum length of long_activations value.
 * @return length of long_activations value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SwitchInterface::maxlenof_long_activations() const
{
  return 1;
}

/** Set long_activations value.
 * 
      Number of consecutive short clicks (turned on). Can be used to recognize
      patterns of clicks. This is an optional field.
    
 * @param new_long_activations new long_activations value
 */
void
SwitchInterface::set_long_activations(const unsigned int new_long_activations)
{
  data->long_activations = new_long_activations;
}

/** Get activation_count value.
 * 
      Number that is to be incremented whenever a short or long activation
      happened. Can be used to decide if a change in status happened.
    
 * @return activation_count value
 */
unsigned int
SwitchInterface::activation_count() const
{
  return data->activation_count;
}

/** Get maximum length of activation_count value.
 * @return length of activation_count value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SwitchInterface::maxlenof_activation_count() const
{
  return 1;
}

/** Set activation_count value.
 * 
      Number that is to be incremented whenever a short or long activation
      happened. Can be used to decide if a change in status happened.
    
 * @param new_activation_count new activation_count value
 */
void
SwitchInterface::set_activation_count(const unsigned int new_activation_count)
{
  data->activation_count = new_activation_count;
}

/* =========== message create =========== */
Message *
SwitchInterface::create_message(const char *type) const
{
  if ( strncmp("SetMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetMessage();
  } else if ( strncmp("EnableSwitchMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new EnableSwitchMessage();
  } else if ( strncmp("DisableSwitchMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new DisableSwitchMessage();
  } else if ( strncmp("EnableDurationMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new EnableDurationMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
SwitchInterface::copy_values(const Interface *other)
{
  const SwitchInterface *oi = dynamic_cast<const SwitchInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(SwitchInterface_data_t));
}

const char *
SwitchInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class SwitchInterface::SetMessage <interfaces/SwitchInterface.h>
 * SetMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_enabled initial value for enabled
 * @param ini_value initial value for value
 */
SwitchInterface::SetMessage::SetMessage(const bool ini_enabled, const float ini_value) : Message("SetMessage")
{
  data_size = sizeof(SetMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMessage_data_t *)data_ptr;
  data->enabled = ini_enabled;
  data->value = ini_value;
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
}
/** Constructor */
SwitchInterface::SetMessage::SetMessage() : Message("SetMessage")
{
  data_size = sizeof(SetMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMessage_data_t *)data_ptr;
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
}

/** Destructor */
SwitchInterface::SetMessage::~SetMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SwitchInterface::SetMessage::SetMessage(const SetMessage *m) : Message("SetMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetMessage_data_t *)data_ptr;
}

/* Methods */
/** Get enabled value.
 * 
      True if the switch is currently enabled.
    
 * @return enabled value
 */
bool
SwitchInterface::SetMessage::is_enabled() const
{
  return data->enabled;
}

/** Get maximum length of enabled value.
 * @return length of enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SwitchInterface::SetMessage::maxlenof_enabled() const
{
  return 1;
}

/** Set enabled value.
 * 
      True if the switch is currently enabled.
    
 * @param new_enabled new enabled value
 */
void
SwitchInterface::SetMessage::set_enabled(const bool new_enabled)
{
  data->enabled = new_enabled;
}

/** Get value value.
 * 
      If switches support multiple states these can be indicated with
      this value. For example for a switch that notes the intensity it
      could be a value in the valid range.
    
 * @return value value
 */
float
SwitchInterface::SetMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SwitchInterface::SetMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * 
      If switches support multiple states these can be indicated with
      this value. For example for a switch that notes the intensity it
      could be a value in the valid range.
    
 * @param new_value new value value
 */
void
SwitchInterface::SetMessage::set_value(const float new_value)
{
  data->value = new_value;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SwitchInterface::SetMessage::clone() const
{
  return new SwitchInterface::SetMessage(this);
}
/** @class SwitchInterface::EnableSwitchMessage <interfaces/SwitchInterface.h>
 * EnableSwitchMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SwitchInterface::EnableSwitchMessage::EnableSwitchMessage() : Message("EnableSwitchMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
SwitchInterface::EnableSwitchMessage::~EnableSwitchMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
SwitchInterface::EnableSwitchMessage::EnableSwitchMessage(const EnableSwitchMessage *m) : Message("EnableSwitchMessage")
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
SwitchInterface::EnableSwitchMessage::clone() const
{
  return new SwitchInterface::EnableSwitchMessage(this);
}
/** @class SwitchInterface::DisableSwitchMessage <interfaces/SwitchInterface.h>
 * DisableSwitchMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SwitchInterface::DisableSwitchMessage::DisableSwitchMessage() : Message("DisableSwitchMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
SwitchInterface::DisableSwitchMessage::~DisableSwitchMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
SwitchInterface::DisableSwitchMessage::DisableSwitchMessage(const DisableSwitchMessage *m) : Message("DisableSwitchMessage")
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
SwitchInterface::DisableSwitchMessage::clone() const
{
  return new SwitchInterface::DisableSwitchMessage(this);
}
/** @class SwitchInterface::EnableDurationMessage <interfaces/SwitchInterface.h>
 * EnableDurationMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_duration initial value for duration
 * @param ini_value initial value for value
 */
SwitchInterface::EnableDurationMessage::EnableDurationMessage(const float ini_duration, const float ini_value) : Message("EnableDurationMessage")
{
  data_size = sizeof(EnableDurationMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (EnableDurationMessage_data_t *)data_ptr;
  data->duration = ini_duration;
  data->value = ini_value;
  add_fieldinfo(IFT_FLOAT, "duration", 1, &data->duration);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
}
/** Constructor */
SwitchInterface::EnableDurationMessage::EnableDurationMessage() : Message("EnableDurationMessage")
{
  data_size = sizeof(EnableDurationMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (EnableDurationMessage_data_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "duration", 1, &data->duration);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
}

/** Destructor */
SwitchInterface::EnableDurationMessage::~EnableDurationMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SwitchInterface::EnableDurationMessage::EnableDurationMessage(const EnableDurationMessage *m) : Message("EnableDurationMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (EnableDurationMessage_data_t *)data_ptr;
}

/* Methods */
/** Get duration value.
 * Duration in seconds for which
    the switch should be enabled.
 * @return duration value
 */
float
SwitchInterface::EnableDurationMessage::duration() const
{
  return data->duration;
}

/** Get maximum length of duration value.
 * @return length of duration value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SwitchInterface::EnableDurationMessage::maxlenof_duration() const
{
  return 1;
}

/** Set duration value.
 * Duration in seconds for which
    the switch should be enabled.
 * @param new_duration new duration value
 */
void
SwitchInterface::EnableDurationMessage::set_duration(const float new_duration)
{
  data->duration = new_duration;
}

/** Get value value.
 * 
      If switches support multiple states these can be indicated with
      this value. For example for a switch that notes the intensity it
      could be a value in the valid range.
    
 * @return value value
 */
float
SwitchInterface::EnableDurationMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SwitchInterface::EnableDurationMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * 
      If switches support multiple states these can be indicated with
      this value. For example for a switch that notes the intensity it
      could be a value in the valid range.
    
 * @param new_value new value value
 */
void
SwitchInterface::EnableDurationMessage::set_value(const float new_value)
{
  data->value = new_value;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SwitchInterface::EnableDurationMessage::clone() const
{
  return new SwitchInterface::EnableDurationMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
SwitchInterface::message_valid(const Message *message) const
{
  const SetMessage *m0 = dynamic_cast<const SetMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const EnableSwitchMessage *m1 = dynamic_cast<const EnableSwitchMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const DisableSwitchMessage *m2 = dynamic_cast<const DisableSwitchMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const EnableDurationMessage *m3 = dynamic_cast<const EnableDurationMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(SwitchInterface)
/// @endcond


} // end namespace fawkes
