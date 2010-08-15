
/***************************************************************************
 *  SpeechSynthInterface.cpp - Fawkes BlackBoard Interface - SpeechSynthInterface
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

#include <interfaces/SpeechSynthInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class SpeechSynthInterface <interfaces/SpeechSynthInterface.h>
 * SpeechSynthInterface Fawkes BlackBoard Interface.
 * 
      The interface provides access to a spech synthesizer facility.
      On systems that support this feature strings can be ordered for
      synthesis and audio output. Multiple messages ordering speech
      should be enqueued and processed one after another by providers.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
SpeechSynthInterface::SpeechSynthInterface() : Interface()
{
  data_size = sizeof(SpeechSynthInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (SpeechSynthInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "text", 1024, data->text);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_FLOAT, "duration", 1, &data->duration);
  add_messageinfo("SayMessage");
  unsigned char tmp_hash[] = {0x28, 0x11, 0x46, 0x87, 0xb1, 0x65, 0x92, 0x96, 0xe6, 0x6e, 0x18, 0x8a, 0xdc, 0x8, 0xb0, 0x69};
  set_hash(tmp_hash);
}

/** Destructor */
SpeechSynthInterface::~SpeechSynthInterface()
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
SpeechSynthInterface::text() const
{
  return data->text;
}

/** Get maximum length of text value.
 * @return length of text value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SpeechSynthInterface::maxlenof_text() const
{
  return 1024;
}

/** Set text value.
 * 
      Last spoken string. Must be properly null-terminated.
    
 * @param new_text new text value
 */
void
SpeechSynthInterface::set_text(const char * new_text)
{
  strncpy(data->text, new_text, sizeof(data->text));
  data_changed = true;
}

/** Get msgid value.
 * 
      The ID of the message that is currently being processed,
      or 0 if no message is being processed.
    
 * @return msgid value
 */
uint32_t
SpeechSynthInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SpeechSynthInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * 
      The ID of the message that is currently being processed,
      or 0 if no message is being processed.
    
 * @param new_msgid new msgid value
 */
void
SpeechSynthInterface::set_msgid(const uint32_t new_msgid)
{
  data->msgid = new_msgid;
  data_changed = true;
}

/** Get final value.
 * 
      True, if the last text has been spoken, false if it is still running.
    
 * @return final value
 */
bool
SpeechSynthInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SpeechSynthInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * 
      True, if the last text has been spoken, false if it is still running.
    
 * @param new_final new final value
 */
void
SpeechSynthInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/** Get duration value.
 * 
      Length in seconds that it takes to speek the current text, -1 if
      unknown. This is the total duration of the current string, *not* the
      duration of already spoken or yet to speak text!
    
 * @return duration value
 */
float
SpeechSynthInterface::duration() const
{
  return data->duration;
}

/** Get maximum length of duration value.
 * @return length of duration value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SpeechSynthInterface::maxlenof_duration() const
{
  return 1;
}

/** Set duration value.
 * 
      Length in seconds that it takes to speek the current text, -1 if
      unknown. This is the total duration of the current string, *not* the
      duration of already spoken or yet to speak text!
    
 * @param new_duration new duration value
 */
void
SpeechSynthInterface::set_duration(const float new_duration)
{
  data->duration = new_duration;
  data_changed = true;
}

/* =========== message create =========== */
Message *
SpeechSynthInterface::create_message(const char *type) const
{
  if ( strncmp("SayMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SayMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
SpeechSynthInterface::copy_values(const Interface *other)
{
  const SpeechSynthInterface *oi = dynamic_cast<const SpeechSynthInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(SpeechSynthInterface_data_t));
}

const char *
SpeechSynthInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class SpeechSynthInterface::SayMessage <interfaces/SpeechSynthInterface.h>
 * SayMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_text initial value for text
 */
SpeechSynthInterface::SayMessage::SayMessage(const char * ini_text) : Message("SayMessage")
{
  data_size = sizeof(SayMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SayMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->text, ini_text, 1024);
  add_fieldinfo(IFT_STRING, "text", 1024, data->text);
}
/** Constructor */
SpeechSynthInterface::SayMessage::SayMessage() : Message("SayMessage")
{
  data_size = sizeof(SayMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SayMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "text", 1024, data->text);
}

/** Destructor */
SpeechSynthInterface::SayMessage::~SayMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SpeechSynthInterface::SayMessage::SayMessage(const SayMessage *m) : Message("SayMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SayMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get text value.
 * 
      Last spoken string. Must be properly null-terminated.
    
 * @return text value
 */
char *
SpeechSynthInterface::SayMessage::text() const
{
  return data->text;
}

/** Get maximum length of text value.
 * @return length of text value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SpeechSynthInterface::SayMessage::maxlenof_text() const
{
  return 1024;
}

/** Set text value.
 * 
      Last spoken string. Must be properly null-terminated.
    
 * @param new_text new text value
 */
void
SpeechSynthInterface::SayMessage::set_text(const char * new_text)
{
  strncpy(data->text, new_text, sizeof(data->text));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SpeechSynthInterface::SayMessage::clone() const
{
  return new SpeechSynthInterface::SayMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
SpeechSynthInterface::message_valid(const Message *message) const
{
  const SayMessage *m0 = dynamic_cast<const SayMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(SpeechSynthInterface)
/// @endcond


} // end namespace fawkes
