
/***************************************************************************
 *  PddlPlannerInterface.cpp - Fawkes BlackBoard Interface - PddlPlannerInterface
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

#include <interfaces/PddlPlannerInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class PddlPlannerInterface <interfaces/PddlPlannerInterface.h>
 * PddlPlannerInterface Fawkes BlackBoard Interface.
 * 
      Interface to start a PDDL planner and parse the resulting plan
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
PddlPlannerInterface::PddlPlannerInterface() : Interface()
{
  data_size = sizeof(PddlPlannerInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (PddlPlannerInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "msg_id", 1, &data->msg_id);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_messageinfo("PlanMessage");
  unsigned char tmp_hash[] = {0x1, 0xbc, 0xa0, 0xb1, 0x9b, 0xa6, 0xc6, 0xa, 0x69, 0xf2, 0x65, 0x38, 0x5a, 0x93, 0x48, 0xf9};
  set_hash(tmp_hash);
}

/** Destructor */
PddlPlannerInterface::~PddlPlannerInterface()
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
PddlPlannerInterface::msg_id() const
{
  return data->msg_id;
}

/** Get maximum length of msg_id value.
 * @return length of msg_id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PddlPlannerInterface::maxlenof_msg_id() const
{
  return 1;
}

/** Set msg_id value.
 * 
      The ID of the message that was processed last
    
 * @param new_msg_id new msg_id value
 */
void
PddlPlannerInterface::set_msg_id(const uint32_t new_msg_id)
{
  data->msg_id = new_msg_id;
  data_changed = true;
}

/** Get final value.
 * 
      Is the planning finished?
    
 * @return final value
 */
bool
PddlPlannerInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PddlPlannerInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * 
      Is the planning finished?
    
 * @param new_final new final value
 */
void
PddlPlannerInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/* =========== message create =========== */
Message *
PddlPlannerInterface::create_message(const char *type) const
{
  if ( strncmp("PlanMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new PlanMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
PddlPlannerInterface::copy_values(const Interface *other)
{
  const PddlPlannerInterface *oi = dynamic_cast<const PddlPlannerInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(PddlPlannerInterface_data_t));
}

const char *
PddlPlannerInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class PddlPlannerInterface::PlanMessage <interfaces/PddlPlannerInterface.h>
 * PlanMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_dummy initial value for dummy
 */
PddlPlannerInterface::PlanMessage::PlanMessage(const char * ini_dummy) : Message("PlanMessage")
{
  data_size = sizeof(PlanMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PlanMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->dummy, ini_dummy, 1);
  add_fieldinfo(IFT_STRING, "dummy", 1, data->dummy);
}
/** Constructor */
PddlPlannerInterface::PlanMessage::PlanMessage() : Message("PlanMessage")
{
  data_size = sizeof(PlanMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PlanMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "dummy", 1, data->dummy);
}

/** Destructor */
PddlPlannerInterface::PlanMessage::~PlanMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PddlPlannerInterface::PlanMessage::PlanMessage(const PlanMessage *m) : Message("PlanMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (PlanMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get dummy value.
 * 
    	Just a dummy field.
    
 * @return dummy value
 */
char *
PddlPlannerInterface::PlanMessage::dummy() const
{
  return data->dummy;
}

/** Get maximum length of dummy value.
 * @return length of dummy value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PddlPlannerInterface::PlanMessage::maxlenof_dummy() const
{
  return 1;
}

/** Set dummy value.
 * 
    	Just a dummy field.
    
 * @param new_dummy new dummy value
 */
void
PddlPlannerInterface::PlanMessage::set_dummy(const char * new_dummy)
{
  strncpy(data->dummy, new_dummy, sizeof(data->dummy));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PddlPlannerInterface::PlanMessage::clone() const
{
  return new PddlPlannerInterface::PlanMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
PddlPlannerInterface::message_valid(const Message *message) const
{
  const PlanMessage *m0 = dynamic_cast<const PlanMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(PddlPlannerInterface)
/// @endcond


} // end namespace fawkes
