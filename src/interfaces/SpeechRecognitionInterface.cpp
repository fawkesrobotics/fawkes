
/***************************************************************************
 *  SpeechRecognitionInterface.cpp - Fawkes BlackBoard Interface - SpeechRecognitionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2009  Tim Niemueller and Masrur Doostdar
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

#include <interfaces/SpeechRecognitionInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class SpeechRecognitionInterface <interfaces/SpeechRecognitionInterface.h>
 * SpeechRecognitionInterface Fawkes BlackBoard Interface.
 * 
      The interface provides access to a spech recognition facility.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
SpeechRecognitionInterface::SpeechRecognitionInterface() : Interface()
{
  data_size = sizeof(SpeechRecognitionInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (SpeechRecognitionInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "text", 1024, data->text);
  add_fieldinfo(IFT_UINT32, "counter", 1, &data->counter);
  add_fieldinfo(IFT_BOOL, "processing", 1, &data->processing);
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
  add_messageinfo("ResetMessage");
  add_messageinfo("SetEnabledMessage");
  unsigned char tmp_hash[] = {0x8f, 0x5c, 0xd, 0x42, 0x1b, 0x22, 0x75, 0x3d, 0x50, 0x66, 0x70, 0x8, 0x1f, 0x47, 0xa7, 0xfd};
  set_hash(tmp_hash);
}

/** Destructor */
SpeechRecognitionInterface::~SpeechRecognitionInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get text value.
 * 
      Last spoken string. Must be properly null-terminated.
    
 * @return text value
 */
char *
SpeechRecognitionInterface::text() const
{
  return data->text;
}

/** Get maximum length of text value.
 * @return length of text value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SpeechRecognitionInterface::maxlenof_text() const
{
  return 1024;
}

/** Set text value.
 * 
      Last spoken string. Must be properly null-terminated.
    
 * @param new_text new text value
 */
void
SpeechRecognitionInterface::set_text(const char * new_text)
{
  strncpy(data->text, new_text, sizeof(data->text));
  data_changed = true;
}

/** Get counter value.
 * 
      Counter for messages. Increased after each new recognized string.
    
 * @return counter value
 */
uint32_t
SpeechRecognitionInterface::counter() const
{
  return data->counter;
}

/** Get maximum length of counter value.
 * @return length of counter value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SpeechRecognitionInterface::maxlenof_counter() const
{
  return 1;
}

/** Set counter value.
 * 
      Counter for messages. Increased after each new recognized string.
    
 * @param new_counter new counter value
 */
void
SpeechRecognitionInterface::set_counter(const uint32_t new_counter)
{
  data->counter = new_counter;
  data_changed = true;
}

/** Get processing value.
 * 
      True, if the the speech recognition is currently processing.
    
 * @return processing value
 */
bool
SpeechRecognitionInterface::is_processing() const
{
  return data->processing;
}

/** Get maximum length of processing value.
 * @return length of processing value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SpeechRecognitionInterface::maxlenof_processing() const
{
  return 1;
}

/** Set processing value.
 * 
      True, if the the speech recognition is currently processing.
    
 * @param new_processing new processing value
 */
void
SpeechRecognitionInterface::set_processing(const bool new_processing)
{
  data->processing = new_processing;
  data_changed = true;
}

/** Get enabled value.
 * 
      True, if speech processing is currently enabled, false otherwise.
    
 * @return enabled value
 */
bool
SpeechRecognitionInterface::is_enabled() const
{
  return data->enabled;
}

/** Get maximum length of enabled value.
 * @return length of enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SpeechRecognitionInterface::maxlenof_enabled() const
{
  return 1;
}

/** Set enabled value.
 * 
      True, if speech processing is currently enabled, false otherwise.
    
 * @param new_enabled new enabled value
 */
void
SpeechRecognitionInterface::set_enabled(const bool new_enabled)
{
  data->enabled = new_enabled;
  data_changed = true;
}

/* =========== message create =========== */
Message *
SpeechRecognitionInterface::create_message(const char *type) const
{
  if ( strncmp("ResetMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ResetMessage();
  } else if ( strncmp("SetEnabledMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetEnabledMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
SpeechRecognitionInterface::copy_values(const Interface *other)
{
  const SpeechRecognitionInterface *oi = dynamic_cast<const SpeechRecognitionInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(SpeechRecognitionInterface_data_t));
}

const char *
SpeechRecognitionInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class SpeechRecognitionInterface::ResetMessage <interfaces/SpeechRecognitionInterface.h>
 * ResetMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SpeechRecognitionInterface::ResetMessage::ResetMessage() : Message("ResetMessage")
{
  data_size = sizeof(ResetMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ResetMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
SpeechRecognitionInterface::ResetMessage::~ResetMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SpeechRecognitionInterface::ResetMessage::ResetMessage(const ResetMessage *m) : Message("ResetMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ResetMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SpeechRecognitionInterface::ResetMessage::clone() const
{
  return new SpeechRecognitionInterface::ResetMessage(this);
}
/** @class SpeechRecognitionInterface::SetEnabledMessage <interfaces/SpeechRecognitionInterface.h>
 * SetEnabledMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_enabled initial value for enabled
 */
SpeechRecognitionInterface::SetEnabledMessage::SetEnabledMessage(const bool ini_enabled) : Message("SetEnabledMessage")
{
  data_size = sizeof(SetEnabledMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->enabled = ini_enabled;
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
}
/** Constructor */
SpeechRecognitionInterface::SetEnabledMessage::SetEnabledMessage() : Message("SetEnabledMessage")
{
  data_size = sizeof(SetEnabledMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
}

/** Destructor */
SpeechRecognitionInterface::SetEnabledMessage::~SetEnabledMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SpeechRecognitionInterface::SetEnabledMessage::SetEnabledMessage(const SetEnabledMessage *m) : Message("SetEnabledMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get enabled value.
 * 
      True, if speech processing is currently enabled, false otherwise.
    
 * @return enabled value
 */
bool
SpeechRecognitionInterface::SetEnabledMessage::is_enabled() const
{
  return data->enabled;
}

/** Get maximum length of enabled value.
 * @return length of enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SpeechRecognitionInterface::SetEnabledMessage::maxlenof_enabled() const
{
  return 1;
}

/** Set enabled value.
 * 
      True, if speech processing is currently enabled, false otherwise.
    
 * @param new_enabled new enabled value
 */
void
SpeechRecognitionInterface::SetEnabledMessage::set_enabled(const bool new_enabled)
{
  data->enabled = new_enabled;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SpeechRecognitionInterface::SetEnabledMessage::clone() const
{
  return new SpeechRecognitionInterface::SetEnabledMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
SpeechRecognitionInterface::message_valid(const Message *message) const
{
  const ResetMessage *m0 = dynamic_cast<const ResetMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SetEnabledMessage *m1 = dynamic_cast<const SetEnabledMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(SpeechRecognitionInterface)
/// @endcond


} // end namespace fawkes
