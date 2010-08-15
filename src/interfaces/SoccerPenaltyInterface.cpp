
/***************************************************************************
 *  SoccerPenaltyInterface.cpp - Fawkes BlackBoard Interface - SoccerPenaltyInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008-2010  Tim Niemueller
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

#include <interfaces/SoccerPenaltyInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class SoccerPenaltyInterface <interfaces/SoccerPenaltyInterface.h>
 * SoccerPenaltyInterface Fawkes BlackBoard Interface.
 * 
      This interface stores penalization information for soccer robots.
      Currently it contains constants used in the RoboCup Standard Platform
      League (SPL).
    
 * @ingroup FawkesInterfaces
 */


/** SPL_PENALTY_NONE constant */
const uint16_t SoccerPenaltyInterface::SPL_PENALTY_NONE = 0;
/** SPL_PENALTY_BALL_HOLDING constant */
const uint16_t SoccerPenaltyInterface::SPL_PENALTY_BALL_HOLDING = 1;
/** SPL_PENALTY_PLAYER_PUSHING constant */
const uint16_t SoccerPenaltyInterface::SPL_PENALTY_PLAYER_PUSHING = 2;
/** SPL_PENALTY_OBSTRUCTION constant */
const uint16_t SoccerPenaltyInterface::SPL_PENALTY_OBSTRUCTION = 3;
/** SPL_PENALTY_INACTIVE_PLAYER constant */
const uint16_t SoccerPenaltyInterface::SPL_PENALTY_INACTIVE_PLAYER = 4;
/** SPL_PENALTY_ILLEGAL_DEFENDER constant */
const uint16_t SoccerPenaltyInterface::SPL_PENALTY_ILLEGAL_DEFENDER = 5;
/** SPL_PENALTY_LEAVING_THE_FIELD constant */
const uint16_t SoccerPenaltyInterface::SPL_PENALTY_LEAVING_THE_FIELD = 6;
/** SPL_PENALTY_PLAYING_WITH_HANDS constant */
const uint16_t SoccerPenaltyInterface::SPL_PENALTY_PLAYING_WITH_HANDS = 7;
/** SPL_PENALTY_REQ_FOR_PICKUP constant */
const uint16_t SoccerPenaltyInterface::SPL_PENALTY_REQ_FOR_PICKUP = 8;
/** SPL_PENALTY_MANUAL constant */
const uint16_t SoccerPenaltyInterface::SPL_PENALTY_MANUAL = 15;

/** Constructor */
SoccerPenaltyInterface::SoccerPenaltyInterface() : Interface()
{
  data_size = sizeof(SoccerPenaltyInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (SoccerPenaltyInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT16, "penalty", 1, &data->penalty);
  add_fieldinfo(IFT_UINT16, "remaining", 1, &data->remaining);
  add_messageinfo("SetPenaltyMessage");
  unsigned char tmp_hash[] = {0xa0, 0xa1, 0xf0, 0xc2, 0x4e, 0x8c, 0xd1, 0xe1, 0xaf, 0x46, 0x11, 0xe9, 0xa0, 0xc8, 0xaf, 0x5d};
  set_hash(tmp_hash);
}

/** Destructor */
SoccerPenaltyInterface::~SoccerPenaltyInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get penalty value.
 * Current penalty code.
 * @return penalty value
 */
uint16_t
SoccerPenaltyInterface::penalty() const
{
  return data->penalty;
}

/** Get maximum length of penalty value.
 * @return length of penalty value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SoccerPenaltyInterface::maxlenof_penalty() const
{
  return 1;
}

/** Set penalty value.
 * Current penalty code.
 * @param new_penalty new penalty value
 */
void
SoccerPenaltyInterface::set_penalty(const uint16_t new_penalty)
{
  data->penalty = new_penalty;
  data_changed = true;
}

/** Get remaining value.
 * Estimated time in seconds until the robot is unpenalized.
 * @return remaining value
 */
uint16_t
SoccerPenaltyInterface::remaining() const
{
  return data->remaining;
}

/** Get maximum length of remaining value.
 * @return length of remaining value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SoccerPenaltyInterface::maxlenof_remaining() const
{
  return 1;
}

/** Set remaining value.
 * Estimated time in seconds until the robot is unpenalized.
 * @param new_remaining new remaining value
 */
void
SoccerPenaltyInterface::set_remaining(const uint16_t new_remaining)
{
  data->remaining = new_remaining;
  data_changed = true;
}

/* =========== message create =========== */
Message *
SoccerPenaltyInterface::create_message(const char *type) const
{
  if ( strncmp("SetPenaltyMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetPenaltyMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
SoccerPenaltyInterface::copy_values(const Interface *other)
{
  const SoccerPenaltyInterface *oi = dynamic_cast<const SoccerPenaltyInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(SoccerPenaltyInterface_data_t));
}

const char *
SoccerPenaltyInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class SoccerPenaltyInterface::SetPenaltyMessage <interfaces/SoccerPenaltyInterface.h>
 * SetPenaltyMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_penalty initial value for penalty
 */
SoccerPenaltyInterface::SetPenaltyMessage::SetPenaltyMessage(const uint16_t ini_penalty) : Message("SetPenaltyMessage")
{
  data_size = sizeof(SetPenaltyMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPenaltyMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->penalty = ini_penalty;
  add_fieldinfo(IFT_UINT16, "penalty", 1, &data->penalty);
}
/** Constructor */
SoccerPenaltyInterface::SetPenaltyMessage::SetPenaltyMessage() : Message("SetPenaltyMessage")
{
  data_size = sizeof(SetPenaltyMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPenaltyMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT16, "penalty", 1, &data->penalty);
}

/** Destructor */
SoccerPenaltyInterface::SetPenaltyMessage::~SetPenaltyMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SoccerPenaltyInterface::SetPenaltyMessage::SetPenaltyMessage(const SetPenaltyMessage *m) : Message("SetPenaltyMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetPenaltyMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get penalty value.
 * Current penalty code.
 * @return penalty value
 */
uint16_t
SoccerPenaltyInterface::SetPenaltyMessage::penalty() const
{
  return data->penalty;
}

/** Get maximum length of penalty value.
 * @return length of penalty value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SoccerPenaltyInterface::SetPenaltyMessage::maxlenof_penalty() const
{
  return 1;
}

/** Set penalty value.
 * Current penalty code.
 * @param new_penalty new penalty value
 */
void
SoccerPenaltyInterface::SetPenaltyMessage::set_penalty(const uint16_t new_penalty)
{
  data->penalty = new_penalty;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SoccerPenaltyInterface::SetPenaltyMessage::clone() const
{
  return new SoccerPenaltyInterface::SetPenaltyMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
SoccerPenaltyInterface::message_valid(const Message *message) const
{
  const SetPenaltyMessage *m0 = dynamic_cast<const SetPenaltyMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(SoccerPenaltyInterface)
/// @endcond


} // end namespace fawkes
