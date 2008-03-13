
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
  unsigned char tmp_hash[] = {0xf1, 0x86, 0xd3, 0xc4, 0xd8, 0x3d, 0xb7, 0x18, 0xc2, 0x12, 0x54, 0x87, 00, 0xd7, 0xf7, 0xbe};
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
SkillerInterface::skill_string()
{
  return data->skill_string;
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

/* =========== message create =========== */
Message *
SkillerInterface::create_message(const char *type) const
{
  if ( strncmp("CallSkillMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CallSkillMessage();
  } else if ( strncmp("RestartInterpreterMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RestartInterpreterMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/* =========== messages =========== */
/** @class SkillerInterface::CallSkillMessage interfaces/skiller.h
 * CallSkillMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_skill_string initial value for skill_string
 */
SkillerInterface::CallSkillMessage::CallSkillMessage(char * ini_skill_string) : Message("CallSkillMessage")
{
  data_size = sizeof(CallSkillMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CallSkillMessage_data_t *)data_ptr;
  strncpy(data->skill_string, ini_skill_string, 1024);
}
/** Constructor */
SkillerInterface::CallSkillMessage::CallSkillMessage() : Message("CallSkillMessage")
{
  data_size = sizeof(CallSkillMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CallSkillMessage_data_t *)data_ptr;
}

/** Destructor */
SkillerInterface::CallSkillMessage::~CallSkillMessage()
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
SkillerInterface::CallSkillMessage::skill_string()
{
  return data->skill_string;
}

/** Set skill_string value.
 * 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
    
 * @param new_skill_string new skill_string value
 */
void
SkillerInterface::CallSkillMessage::set_skill_string(const char * new_skill_string)
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
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
SkillerInterface::message_valid(const Message *message) const
{
  const CallSkillMessage *m0 = dynamic_cast<const CallSkillMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const RestartInterpreterMessage *m1 = dynamic_cast<const RestartInterpreterMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(SkillerInterface)
/// @endcond

