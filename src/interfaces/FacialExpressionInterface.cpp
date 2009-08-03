
/***************************************************************************
 *  FacialExpressionInterface.cpp - Fawkes BlackBoard Interface - FacialExpressionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2009  Bahram Maleki-Fard
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

#include <interfaces/FacialExpressionInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class FacialExpressionInterface <interfaces/FacialExpressionInterface.h>
 * FacialExpressionInterface Fawkes BlackBoard Interface.
 * 
      Interface to acces facial expressions on display (RCSoft)
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
FacialExpressionInterface::FacialExpressionInterface() : Interface()
{
  data_size = sizeof(FacialExpressionInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (FacialExpressionInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_messageinfo("MoveBrowsMessage");
  add_messageinfo("MoveEyesMessage");
  add_messageinfo("MoveJowlMessage");
  add_messageinfo("MoveMouthMessage");
  unsigned char tmp_hash[] = {0x1, 0xbd, 0xc6, 0x65, 0xb3, 0x10, 0xcb, 0x5f, 0xe8, 0x78, 0xdd, 0x6, 0xe, 0x82, 0x7f, 0x80};
  set_hash(tmp_hash);
}

/** Destructor */
FacialExpressionInterface::~FacialExpressionInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get brows_action value.
 * Type of action of brows
 * @return brows_action value
 */
FacialExpressionInterface::brows_t
FacialExpressionInterface::brows_action() const
{
  return data->brows_action;
}

/** Get maximum length of brows_action value.
 * @return length of brows_action value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacialExpressionInterface::maxlenof_brows_action() const
{
  return 1;
}

/** Set brows_action value.
 * Type of action of brows
 * @param new_brows_action new brows_action value
 */
void
FacialExpressionInterface::set_brows_action(const brows_t new_brows_action)
{
  data->brows_action = new_brows_action;
}

/** Get eyes_action value.
 * Type of action of eyes
 * @return eyes_action value
 */
FacialExpressionInterface::eyes_t
FacialExpressionInterface::eyes_action() const
{
  return data->eyes_action;
}

/** Get maximum length of eyes_action value.
 * @return length of eyes_action value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacialExpressionInterface::maxlenof_eyes_action() const
{
  return 1;
}

/** Set eyes_action value.
 * Type of action of eyes
 * @param new_eyes_action new eyes_action value
 */
void
FacialExpressionInterface::set_eyes_action(const eyes_t new_eyes_action)
{
  data->eyes_action = new_eyes_action;
}

/** Get jowl_action value.
 * Type of action of jown
 * @return jowl_action value
 */
FacialExpressionInterface::jowl_t
FacialExpressionInterface::jowl_action() const
{
  return data->jowl_action;
}

/** Get maximum length of jowl_action value.
 * @return length of jowl_action value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacialExpressionInterface::maxlenof_jowl_action() const
{
  return 1;
}

/** Set jowl_action value.
 * Type of action of jown
 * @param new_jowl_action new jowl_action value
 */
void
FacialExpressionInterface::set_jowl_action(const jowl_t new_jowl_action)
{
  data->jowl_action = new_jowl_action;
}

/** Get mouth_action value.
 * Type of action of mouth
 * @return mouth_action value
 */
FacialExpressionInterface::mouth_t
FacialExpressionInterface::mouth_action() const
{
  return data->mouth_action;
}

/** Get maximum length of mouth_action value.
 * @return length of mouth_action value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacialExpressionInterface::maxlenof_mouth_action() const
{
  return 1;
}

/** Set mouth_action value.
 * Type of action of mouth
 * @param new_mouth_action new mouth_action value
 */
void
FacialExpressionInterface::set_mouth_action(const mouth_t new_mouth_action)
{
  data->mouth_action = new_mouth_action;
}

/* =========== message create =========== */
Message *
FacialExpressionInterface::create_message(const char *type) const
{
  if ( strncmp("MoveBrowsMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new MoveBrowsMessage();
  } else if ( strncmp("MoveEyesMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new MoveEyesMessage();
  } else if ( strncmp("MoveJowlMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new MoveJowlMessage();
  } else if ( strncmp("MoveMouthMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new MoveMouthMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
FacialExpressionInterface::copy_values(const Interface *other)
{
  const FacialExpressionInterface *oi = dynamic_cast<const FacialExpressionInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(FacialExpressionInterface_data_t));
}

/* =========== messages =========== */
/** @class FacialExpressionInterface::MoveBrowsMessage <interfaces/FacialExpressionInterface.h>
 * MoveBrowsMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_brows_action initial value for brows_action
 */
FacialExpressionInterface::MoveBrowsMessage::MoveBrowsMessage(const brows_t ini_brows_action) : Message("MoveBrowsMessage")
{
  data_size = sizeof(MoveBrowsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveBrowsMessage_data_t *)data_ptr;
  data->brows_action = ini_brows_action;
}
/** Constructor */
FacialExpressionInterface::MoveBrowsMessage::MoveBrowsMessage() : Message("MoveBrowsMessage")
{
  data_size = sizeof(MoveBrowsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveBrowsMessage_data_t *)data_ptr;
}

/** Destructor */
FacialExpressionInterface::MoveBrowsMessage::~MoveBrowsMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
FacialExpressionInterface::MoveBrowsMessage::MoveBrowsMessage(const MoveBrowsMessage *m) : Message("MoveBrowsMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (MoveBrowsMessage_data_t *)data_ptr;
}

/* Methods */
/** Get brows_action value.
 * Type of action of brows
 * @return brows_action value
 */
FacialExpressionInterface::brows_t
FacialExpressionInterface::MoveBrowsMessage::brows_action() const
{
  return data->brows_action;
}

/** Get maximum length of brows_action value.
 * @return length of brows_action value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacialExpressionInterface::MoveBrowsMessage::maxlenof_brows_action() const
{
  return 1;
}

/** Set brows_action value.
 * Type of action of brows
 * @param new_brows_action new brows_action value
 */
void
FacialExpressionInterface::MoveBrowsMessage::set_brows_action(const brows_t new_brows_action)
{
  data->brows_action = new_brows_action;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
FacialExpressionInterface::MoveBrowsMessage::clone() const
{
  return new FacialExpressionInterface::MoveBrowsMessage(this);
}
/** @class FacialExpressionInterface::MoveEyesMessage <interfaces/FacialExpressionInterface.h>
 * MoveEyesMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_eyes_action initial value for eyes_action
 */
FacialExpressionInterface::MoveEyesMessage::MoveEyesMessage(const eyes_t ini_eyes_action) : Message("MoveEyesMessage")
{
  data_size = sizeof(MoveEyesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveEyesMessage_data_t *)data_ptr;
  data->eyes_action = ini_eyes_action;
}
/** Constructor */
FacialExpressionInterface::MoveEyesMessage::MoveEyesMessage() : Message("MoveEyesMessage")
{
  data_size = sizeof(MoveEyesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveEyesMessage_data_t *)data_ptr;
}

/** Destructor */
FacialExpressionInterface::MoveEyesMessage::~MoveEyesMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
FacialExpressionInterface::MoveEyesMessage::MoveEyesMessage(const MoveEyesMessage *m) : Message("MoveEyesMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (MoveEyesMessage_data_t *)data_ptr;
}

/* Methods */
/** Get eyes_action value.
 * Type of action of eyes
 * @return eyes_action value
 */
FacialExpressionInterface::eyes_t
FacialExpressionInterface::MoveEyesMessage::eyes_action() const
{
  return data->eyes_action;
}

/** Get maximum length of eyes_action value.
 * @return length of eyes_action value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacialExpressionInterface::MoveEyesMessage::maxlenof_eyes_action() const
{
  return 1;
}

/** Set eyes_action value.
 * Type of action of eyes
 * @param new_eyes_action new eyes_action value
 */
void
FacialExpressionInterface::MoveEyesMessage::set_eyes_action(const eyes_t new_eyes_action)
{
  data->eyes_action = new_eyes_action;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
FacialExpressionInterface::MoveEyesMessage::clone() const
{
  return new FacialExpressionInterface::MoveEyesMessage(this);
}
/** @class FacialExpressionInterface::MoveJowlMessage <interfaces/FacialExpressionInterface.h>
 * MoveJowlMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_jowl_action initial value for jowl_action
 */
FacialExpressionInterface::MoveJowlMessage::MoveJowlMessage(const jowl_t ini_jowl_action) : Message("MoveJowlMessage")
{
  data_size = sizeof(MoveJowlMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveJowlMessage_data_t *)data_ptr;
  data->jowl_action = ini_jowl_action;
}
/** Constructor */
FacialExpressionInterface::MoveJowlMessage::MoveJowlMessage() : Message("MoveJowlMessage")
{
  data_size = sizeof(MoveJowlMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveJowlMessage_data_t *)data_ptr;
}

/** Destructor */
FacialExpressionInterface::MoveJowlMessage::~MoveJowlMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
FacialExpressionInterface::MoveJowlMessage::MoveJowlMessage(const MoveJowlMessage *m) : Message("MoveJowlMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (MoveJowlMessage_data_t *)data_ptr;
}

/* Methods */
/** Get jowl_action value.
 * Type of action of jown
 * @return jowl_action value
 */
FacialExpressionInterface::jowl_t
FacialExpressionInterface::MoveJowlMessage::jowl_action() const
{
  return data->jowl_action;
}

/** Get maximum length of jowl_action value.
 * @return length of jowl_action value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacialExpressionInterface::MoveJowlMessage::maxlenof_jowl_action() const
{
  return 1;
}

/** Set jowl_action value.
 * Type of action of jown
 * @param new_jowl_action new jowl_action value
 */
void
FacialExpressionInterface::MoveJowlMessage::set_jowl_action(const jowl_t new_jowl_action)
{
  data->jowl_action = new_jowl_action;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
FacialExpressionInterface::MoveJowlMessage::clone() const
{
  return new FacialExpressionInterface::MoveJowlMessage(this);
}
/** @class FacialExpressionInterface::MoveMouthMessage <interfaces/FacialExpressionInterface.h>
 * MoveMouthMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_mouth_action initial value for mouth_action
 */
FacialExpressionInterface::MoveMouthMessage::MoveMouthMessage(const mouth_t ini_mouth_action) : Message("MoveMouthMessage")
{
  data_size = sizeof(MoveMouthMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveMouthMessage_data_t *)data_ptr;
  data->mouth_action = ini_mouth_action;
}
/** Constructor */
FacialExpressionInterface::MoveMouthMessage::MoveMouthMessage() : Message("MoveMouthMessage")
{
  data_size = sizeof(MoveMouthMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveMouthMessage_data_t *)data_ptr;
}

/** Destructor */
FacialExpressionInterface::MoveMouthMessage::~MoveMouthMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
FacialExpressionInterface::MoveMouthMessage::MoveMouthMessage(const MoveMouthMessage *m) : Message("MoveMouthMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (MoveMouthMessage_data_t *)data_ptr;
}

/* Methods */
/** Get mouth_action value.
 * Type of action of mouth
 * @return mouth_action value
 */
FacialExpressionInterface::mouth_t
FacialExpressionInterface::MoveMouthMessage::mouth_action() const
{
  return data->mouth_action;
}

/** Get maximum length of mouth_action value.
 * @return length of mouth_action value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacialExpressionInterface::MoveMouthMessage::maxlenof_mouth_action() const
{
  return 1;
}

/** Set mouth_action value.
 * Type of action of mouth
 * @param new_mouth_action new mouth_action value
 */
void
FacialExpressionInterface::MoveMouthMessage::set_mouth_action(const mouth_t new_mouth_action)
{
  data->mouth_action = new_mouth_action;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
FacialExpressionInterface::MoveMouthMessage::clone() const
{
  return new FacialExpressionInterface::MoveMouthMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
FacialExpressionInterface::message_valid(const Message *message) const
{
  const MoveBrowsMessage *m0 = dynamic_cast<const MoveBrowsMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const MoveEyesMessage *m1 = dynamic_cast<const MoveEyesMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const MoveJowlMessage *m2 = dynamic_cast<const MoveJowlMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const MoveMouthMessage *m3 = dynamic_cast<const MoveMouthMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(FacialExpressionInterface)
/// @endcond


} // end namespace fawkes
