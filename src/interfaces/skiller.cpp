
/***************************************************************************
 *  skiller.cpp - Fawkes BlackBoard Interface - SkillerInterface
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

#include <interfaces/skiller.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class SkillerInterface <interfaces/skiller.h>
 * SkillerInterface Fawkes BlackBoard Interface.
 * 
      The interface provides access to the skill execution runtime plugin.
      It provides basic status information about skiller and allows for
      calling skills via messages. It can also be used to manually restart
      the Lua interpreter if something is wedged.
    
 */



/** Constructor */
SkillerInterface::SkillerInterface() : Interface()
{
  data_size = sizeof(SkillerInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (SkillerInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(Interface::IFT_STRING, "skill_string", data->skill_string);
  add_fieldinfo(Interface::IFT_UINT, "exclusive_controller", &data->exclusive_controller);
  add_fieldinfo(Interface::IFT_BOOL, "continuous", &data->continuous);
  unsigned char tmp_hash[] = {0x7b, 0xe3, 0xf0, 0xfe, 0x60, 0x4d, 0x22, 0x40, 0x7f, 0x8e, 0x7e, 0x1d, 0x92, 0x9c, 0x83, 0x4c};
  set_hash(tmp_hash);
}

/** Destructor */
SkillerInterface::~SkillerInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get skill_string value.
 * 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
    
 * @return skill_string value
 */
char *
SkillerInterface::skill_string() const
{
  return data->skill_string;
}

/** Get maximum length of skill_string value.
 * @return length of skill_string value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::maxlenof_skill_string() const
{
  return 1024;
}

/** Set skill_string value.
 * 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
    
 * @param new_skill_string new skill_string value
 */
void
SkillerInterface::set_skill_string(const char * new_skill_string)
{
  strncpy(data->skill_string, new_skill_string, sizeof(data->skill_string));
}

/** Get exclusive_controller value.
 * 
      Instance serial of the exclusive controller of the skiller. If this does not
      carry your instance serial your exec messages will be ignored. Aquire control with
      the AquireControlMessage. Make sure you release control before exiting.
    
 * @return exclusive_controller value
 */
unsigned int
SkillerInterface::exclusive_controller() const
{
  return data->exclusive_controller;
}

/** Get maximum length of exclusive_controller value.
 * @return length of exclusive_controller value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::maxlenof_exclusive_controller() const
{
  return 1;
}

/** Set exclusive_controller value.
 * 
      Instance serial of the exclusive controller of the skiller. If this does not
      carry your instance serial your exec messages will be ignored. Aquire control with
      the AquireControlMessage. Make sure you release control before exiting.
    
 * @param new_exclusive_controller new exclusive_controller value
 */
void
SkillerInterface::set_exclusive_controller(const unsigned int new_exclusive_controller)
{
  data->exclusive_controller = new_exclusive_controller;
}

/** Get status value.
 * 
      The status of the current skill execution.
    
 * @return status value
 */
SkillerInterface::SkillStatusEnum
SkillerInterface::status() const
{
  return data->status;
}

/** Get maximum length of status value.
 * @return length of status value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::maxlenof_status() const
{
  return 1;
}

/** Set status value.
 * 
      The status of the current skill execution.
    
 * @param new_status new status value
 */
void
SkillerInterface::set_status(const SkillStatusEnum new_status)
{
  data->status = new_status;
}

/** Get continuous value.
 * 
      True if continuous execution is in progress, false if no skill string is executed
      at all or it is executed one-shot with ExecSkillMessage.
    
 * @return continuous value
 */
bool
SkillerInterface::is_continuous() const
{
  return data->continuous;
}

/** Get maximum length of continuous value.
 * @return length of continuous value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::maxlenof_continuous() const
{
  return 1;
}

/** Set continuous value.
 * 
      True if continuous execution is in progress, false if no skill string is executed
      at all or it is executed one-shot with ExecSkillMessage.
    
 * @param new_continuous new continuous value
 */
void
SkillerInterface::set_continuous(const bool new_continuous)
{
  data->continuous = new_continuous;
}

/* =========== message create =========== */
Message *
SkillerInterface::create_message(const char *type) const
{
  if ( strncmp("ExecSkillMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ExecSkillMessage();
  } else if ( strncmp("ExecSkillContinuousMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ExecSkillContinuousMessage();
  } else if ( strncmp("RestartInterpreterMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RestartInterpreterMessage();
  } else if ( strncmp("StopExecMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopExecMessage();
  } else if ( strncmp("AcquireControlMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AcquireControlMessage();
  } else if ( strncmp("ReleaseControlMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ReleaseControlMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/* =========== messages =========== */
/** @class SkillerInterface::ExecSkillMessage interfaces/skiller.h
 * ExecSkillMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_skill_string initial value for skill_string
 */
SkillerInterface::ExecSkillMessage::ExecSkillMessage(const char * ini_skill_string) : Message("ExecSkillMessage")
{
  data_size = sizeof(ExecSkillMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ExecSkillMessage_data_t *)data_ptr;
  strncpy(data->skill_string, ini_skill_string, 1024);
}
/** Constructor */
SkillerInterface::ExecSkillMessage::ExecSkillMessage() : Message("ExecSkillMessage")
{
  data_size = sizeof(ExecSkillMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ExecSkillMessage_data_t *)data_ptr;
}

/** Destructor */
SkillerInterface::ExecSkillMessage::~ExecSkillMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerInterface::ExecSkillMessage::ExecSkillMessage(const ExecSkillMessage *m) : Message("ExecSkillMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ExecSkillMessage_data_t *)data_ptr;
}

/* Methods */
/** Get skill_string value.
 * 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
    
 * @return skill_string value
 */
char *
SkillerInterface::ExecSkillMessage::skill_string() const
{
  return data->skill_string;
}

/** Get maximum length of skill_string value.
 * @return length of skill_string value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::ExecSkillMessage::maxlenof_skill_string() const
{
  return 1024;
}

/** Set skill_string value.
 * 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
    
 * @param new_skill_string new skill_string value
 */
void
SkillerInterface::ExecSkillMessage::set_skill_string(const char * new_skill_string)
{
  strncpy(data->skill_string, new_skill_string, sizeof(data->skill_string));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SkillerInterface::ExecSkillMessage::clone() const
{
  return new SkillerInterface::ExecSkillMessage(this);
}
/** @class SkillerInterface::ExecSkillContinuousMessage interfaces/skiller.h
 * ExecSkillContinuousMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_skill_string initial value for skill_string
 */
SkillerInterface::ExecSkillContinuousMessage::ExecSkillContinuousMessage(const char * ini_skill_string) : Message("ExecSkillContinuousMessage")
{
  data_size = sizeof(ExecSkillContinuousMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ExecSkillContinuousMessage_data_t *)data_ptr;
  strncpy(data->skill_string, ini_skill_string, 1024);
}
/** Constructor */
SkillerInterface::ExecSkillContinuousMessage::ExecSkillContinuousMessage() : Message("ExecSkillContinuousMessage")
{
  data_size = sizeof(ExecSkillContinuousMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ExecSkillContinuousMessage_data_t *)data_ptr;
}

/** Destructor */
SkillerInterface::ExecSkillContinuousMessage::~ExecSkillContinuousMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerInterface::ExecSkillContinuousMessage::ExecSkillContinuousMessage(const ExecSkillContinuousMessage *m) : Message("ExecSkillContinuousMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ExecSkillContinuousMessage_data_t *)data_ptr;
}

/* Methods */
/** Get skill_string value.
 * 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
    
 * @return skill_string value
 */
char *
SkillerInterface::ExecSkillContinuousMessage::skill_string() const
{
  return data->skill_string;
}

/** Get maximum length of skill_string value.
 * @return length of skill_string value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::ExecSkillContinuousMessage::maxlenof_skill_string() const
{
  return 1024;
}

/** Set skill_string value.
 * 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
    
 * @param new_skill_string new skill_string value
 */
void
SkillerInterface::ExecSkillContinuousMessage::set_skill_string(const char * new_skill_string)
{
  strncpy(data->skill_string, new_skill_string, sizeof(data->skill_string));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SkillerInterface::ExecSkillContinuousMessage::clone() const
{
  return new SkillerInterface::ExecSkillContinuousMessage(this);
}
/** @class SkillerInterface::RestartInterpreterMessage interfaces/skiller.h
 * RestartInterpreterMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SkillerInterface::RestartInterpreterMessage::RestartInterpreterMessage() : Message("RestartInterpreterMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
SkillerInterface::RestartInterpreterMessage::~RestartInterpreterMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerInterface::RestartInterpreterMessage::RestartInterpreterMessage(const RestartInterpreterMessage *m) : Message("RestartInterpreterMessage")
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
SkillerInterface::RestartInterpreterMessage::clone() const
{
  return new SkillerInterface::RestartInterpreterMessage(this);
}
/** @class SkillerInterface::StopExecMessage interfaces/skiller.h
 * StopExecMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SkillerInterface::StopExecMessage::StopExecMessage() : Message("StopExecMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
SkillerInterface::StopExecMessage::~StopExecMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerInterface::StopExecMessage::StopExecMessage(const StopExecMessage *m) : Message("StopExecMessage")
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
SkillerInterface::StopExecMessage::clone() const
{
  return new SkillerInterface::StopExecMessage(this);
}
/** @class SkillerInterface::AcquireControlMessage interfaces/skiller.h
 * AcquireControlMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SkillerInterface::AcquireControlMessage::AcquireControlMessage() : Message("AcquireControlMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
SkillerInterface::AcquireControlMessage::~AcquireControlMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerInterface::AcquireControlMessage::AcquireControlMessage(const AcquireControlMessage *m) : Message("AcquireControlMessage")
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
SkillerInterface::AcquireControlMessage::clone() const
{
  return new SkillerInterface::AcquireControlMessage(this);
}
/** @class SkillerInterface::ReleaseControlMessage interfaces/skiller.h
 * ReleaseControlMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SkillerInterface::ReleaseControlMessage::ReleaseControlMessage() : Message("ReleaseControlMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
SkillerInterface::ReleaseControlMessage::~ReleaseControlMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerInterface::ReleaseControlMessage::ReleaseControlMessage(const ReleaseControlMessage *m) : Message("ReleaseControlMessage")
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
SkillerInterface::ReleaseControlMessage::clone() const
{
  return new SkillerInterface::ReleaseControlMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
SkillerInterface::message_valid(const Message *message) const
{
  const ExecSkillMessage *m0 = dynamic_cast<const ExecSkillMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const ExecSkillContinuousMessage *m1 = dynamic_cast<const ExecSkillContinuousMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const RestartInterpreterMessage *m2 = dynamic_cast<const RestartInterpreterMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const StopExecMessage *m3 = dynamic_cast<const StopExecMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const AcquireControlMessage *m4 = dynamic_cast<const AcquireControlMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const ReleaseControlMessage *m5 = dynamic_cast<const ReleaseControlMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(SkillerInterface)
/// @endcond


} // end namespace fawkes
