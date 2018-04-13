
/***************************************************************************
 *  PddlGenInterface.cpp - Fawkes BlackBoard Interface - PddlGenInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  Frederik Zwilling
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

#include <interfaces/PddlGenInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class PddlGenInterface <interfaces/PddlGenInterface.h>
 * PddlGenInterface Fawkes BlackBoard Interface.
 * 
      Interface to start the PDDL generation
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
PddlGenInterface::PddlGenInterface() : Interface()
{
  data_size = sizeof(PddlGenInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (PddlGenInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "msg_id", 1, &data->msg_id);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_messageinfo("GenerateMessage");
  unsigned char tmp_hash[] = {0x24, 0x2e, 0xeb, 0xd7, 0x1d, 0x5d, 0x15, 0x7f, 0x73, 0xca, 0xa3, 0xf3, 0x74, 0x1, 0x55, 0xc5};
  set_hash(tmp_hash);
}

/** Destructor */
PddlGenInterface::~PddlGenInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get msg_id value.
 * 
      The ID of the message that was processed last
    
 * @return msg_id value
 */
uint32_t
PddlGenInterface::msg_id() const
{
  return data->msg_id;
}

/** Get maximum length of msg_id value.
 * @return length of msg_id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PddlGenInterface::maxlenof_msg_id() const
{
  return 1;
}

/** Set msg_id value.
 * 
      The ID of the message that was processed last
    
 * @param new_msg_id new msg_id value
 */
void
PddlGenInterface::set_msg_id(const uint32_t new_msg_id)
{
  data->msg_id = new_msg_id;
  data_changed = true;
}

/** Get final value.
 * 
      Is the generation finished?
    
 * @return final value
 */
bool
PddlGenInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PddlGenInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * 
      Is the generation finished?
    
 * @param new_final new final value
 */
void
PddlGenInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/* =========== message create =========== */
Message *
PddlGenInterface::create_message(const char *type) const
{
  if ( strncmp("GenerateMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GenerateMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
PddlGenInterface::copy_values(const Interface *other)
{
  const PddlGenInterface *oi = dynamic_cast<const PddlGenInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(PddlGenInterface_data_t));
}

const char *
PddlGenInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class PddlGenInterface::GenerateMessage <interfaces/PddlGenInterface.h>
 * GenerateMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_goal initial value for goal
 */
PddlGenInterface::GenerateMessage::GenerateMessage(const char * ini_goal) : Message("GenerateMessage")
{
  data_size = sizeof(GenerateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GenerateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->goal, ini_goal, 1024-1);
  data->goal[1024-1] = 0;
  add_fieldinfo(IFT_STRING, "goal", 1024, data->goal);
}
/** Constructor */
PddlGenInterface::GenerateMessage::GenerateMessage() : Message("GenerateMessage")
{
  data_size = sizeof(GenerateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GenerateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "goal", 1024, data->goal);
}

/** Destructor */
PddlGenInterface::GenerateMessage::~GenerateMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PddlGenInterface::GenerateMessage::GenerateMessage(const GenerateMessage *m) : Message("GenerateMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GenerateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get goal value.
 * 
    	Optional goal to insert into the template dictionary.
    
 * @return goal value
 */
char *
PddlGenInterface::GenerateMessage::goal() const
{
  return data->goal;
}

/** Get maximum length of goal value.
 * @return length of goal value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PddlGenInterface::GenerateMessage::maxlenof_goal() const
{
  return 1024;
}

/** Set goal value.
 * 
    	Optional goal to insert into the template dictionary.
    
 * @param new_goal new goal value
 */
void
PddlGenInterface::GenerateMessage::set_goal(const char * new_goal)
{
  strncpy(data->goal, new_goal, sizeof(data->goal)-1);
  data->goal[sizeof(data->goal)-1] = 0;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PddlGenInterface::GenerateMessage::clone() const
{
  return new PddlGenInterface::GenerateMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
PddlGenInterface::message_valid(const Message *message) const
{
  const GenerateMessage *m0 = dynamic_cast<const GenerateMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(PddlGenInterface)
/// @endcond


} // end namespace fawkes
