
/***************************************************************************
 *  skiller.cpp - Fawkes BlackBoard Interface - SkillerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
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

#include <interfaces/skiller.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

/** @class SkillerInterface interfaces/skiller.h
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
  unsigned char tmp_hash[] = {0xa5, 0x91, 0x13, 0xe7, 0x9d, 0xc4, 0x85, 0x4c, 0x5, 0x67, 0x82, 0x7f, 0x19, 0xa0, 0x75, 0x14};
  set_hash(tmp_hash);
  add_fieldinfo(Interface::IFT_UINT, "exclusive_controller", &data->exclusive_controller);
  add_fieldinfo(Interface::IFT_BOOL, "final", &data->final);
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
SkillerInterface::skill_string()
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
SkillerInterface::exclusive_controller()
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

/** Get final value.
 * 
      True if the execution of the current skill_string is final. False otherwise.
    
 * @return final value
 */
bool
SkillerInterface::is_final()
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * 
      True if the execution of the current skill_string is final. False otherwise.
    
 * @param new_final new final value
 */
void
SkillerInterface::set_final(const bool new_final)
{
  data->final = new_final;
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

/* Methods */
/** Get skill_string value.
 * 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
    
 * @return skill_string value
 */
char *
SkillerInterface::ExecSkillMessage::skill_string()
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

/* Methods */
/** Get skill_string value.
 * 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
    
 * @return skill_string value
 */
char *
SkillerInterface::ExecSkillContinuousMessage::skill_string()
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

/** @class SkillerInterface::RestartInterpreterMessage interfaces/skiller.h
 * RestartInterpreterMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SkillerInterface::RestartInterpreterMessage::RestartInterpreterMessage() : Message("RestartInterpreterMessage")
{
  data_size = sizeof(RestartInterpreterMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RestartInterpreterMessage_data_t *)data_ptr;
}

/** Destructor */
SkillerInterface::RestartInterpreterMessage::~RestartInterpreterMessage()
{
  free(data_ptr);
}

/* Methods */
/** @class SkillerInterface::StopExecMessage interfaces/skiller.h
 * StopExecMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SkillerInterface::StopExecMessage::StopExecMessage() : Message("StopExecMessage")
{
  data_size = sizeof(StopExecMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopExecMessage_data_t *)data_ptr;
}

/** Destructor */
SkillerInterface::StopExecMessage::~StopExecMessage()
{
  free(data_ptr);
}

/* Methods */
/** @class SkillerInterface::AcquireControlMessage interfaces/skiller.h
 * AcquireControlMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SkillerInterface::AcquireControlMessage::AcquireControlMessage() : Message("AcquireControlMessage")
{
  data_size = sizeof(AcquireControlMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AcquireControlMessage_data_t *)data_ptr;
}

/** Destructor */
SkillerInterface::AcquireControlMessage::~AcquireControlMessage()
{
  free(data_ptr);
}

/* Methods */
/** @class SkillerInterface::ReleaseControlMessage interfaces/skiller.h
 * ReleaseControlMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SkillerInterface::ReleaseControlMessage::ReleaseControlMessage() : Message("ReleaseControlMessage")
{
  data_size = sizeof(ReleaseControlMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ReleaseControlMessage_data_t *)data_ptr;
}

/** Destructor */
SkillerInterface::ReleaseControlMessage::~ReleaseControlMessage()
{
  free(data_ptr);
}

/* Methods */
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

