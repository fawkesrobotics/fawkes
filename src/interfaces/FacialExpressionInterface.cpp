
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
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_ENUM, "brows_action", 1, &data->brows_action, "brows_t");
  add_fieldinfo(IFT_ENUM, "eyes_action", 1, &data->eyes_action, "eyes_t");
  add_fieldinfo(IFT_ENUM, "jowl_action", 1, &data->jowl_action, "jowl_t");
  add_fieldinfo(IFT_ENUM, "mouth_action", 1, &data->mouth_action, "mouth_t");
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
/** Convert brows_t constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
FacialExpressionInterface::tostring_brows_t(brows_t value) const
{
  switch (value) {
  case BROWS_DEFAULT: return "BROWS_DEFAULT";
  case BROWS_FROWN: return "BROWS_FROWN";
  case BROWS_LIFT: return "BROWS_LIFT";
  default: return "UNKNOWN";
  }
}
/** Convert eyes_t constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
FacialExpressionInterface::tostring_eyes_t(eyes_t value) const
{
  switch (value) {
  case EYES_DEFAULT: return "EYES_DEFAULT";
  case EYES_UP: return "EYES_UP";
  case EYES_DOWN: return "EYES_DOWN";
  case EYES_LEFT: return "EYES_LEFT";
  case EYES_RIGHT: return "EYES_RIGHT";
  case EYES_COOL: return "EYES_COOL";
  case EYES_CROSS: return "EYES_CROSS";
  case EYES_HEART: return "EYES_HEART";
  case EYES_DOLLAR: return "EYES_DOLLAR";
  default: return "UNKNOWN";
  }
}
/** Convert jowl_t constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
FacialExpressionInterface::tostring_jowl_t(jowl_t value) const
{
  switch (value) {
  case JOWL_DEFAULT: return "JOWL_DEFAULT";
  case JOWL_BLUSH: return "JOWL_BLUSH";
  case JOWL_TEARS: return "JOWL_TEARS";
  default: return "UNKNOWN";
  }
}
/** Convert mouth_t constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
FacialExpressionInterface::tostring_mouth_t(mouth_t value) const
{
  switch (value) {
  case MOUTH_DEFAULT: return "MOUTH_DEFAULT";
  case MOUTH_OPEN: return "MOUTH_OPEN";
  case MOUTH_CLOSE: return "MOUTH_CLOSE";
  case MOUTH_SMILE: return "MOUTH_SMILE";
  case MOUTH_SCOWL: return "MOUTH_SCOWL";
  default: return "UNKNOWN";
  }
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
  data_changed = true;
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
  data_changed = true;
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
  data_changed = true;
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
  data_changed = true;
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

const char *
FacialExpressionInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "brows_t") == 0) {
    return tostring_brows_t((brows_t)val);
  }
  if (strcmp(enumtype, "eyes_t") == 0) {
    return tostring_eyes_t((eyes_t)val);
  }
  if (strcmp(enumtype, "jowl_t") == 0) {
    return tostring_jowl_t((jowl_t)val);
  }
  if (strcmp(enumtype, "mouth_t") == 0) {
    return tostring_mouth_t((mouth_t)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
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
  data_ts   = (message_data_ts_t *)data_ptr;
  data->brows_action = ini_brows_action;
  add_fieldinfo(IFT_ENUM, "brows_action", 1, &data->brows_action, "brows_t");
}
/** Constructor */
FacialExpressionInterface::MoveBrowsMessage::MoveBrowsMessage() : Message("MoveBrowsMessage")
{
  data_size = sizeof(MoveBrowsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveBrowsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_ENUM, "brows_action", 1, &data->brows_action, "brows_t");
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
  data_ts   = (message_data_ts_t *)data_ptr;
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
  data_ts   = (message_data_ts_t *)data_ptr;
  data->eyes_action = ini_eyes_action;
  add_fieldinfo(IFT_ENUM, "eyes_action", 1, &data->eyes_action, "eyes_t");
}
/** Constructor */
FacialExpressionInterface::MoveEyesMessage::MoveEyesMessage() : Message("MoveEyesMessage")
{
  data_size = sizeof(MoveEyesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveEyesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_ENUM, "eyes_action", 1, &data->eyes_action, "eyes_t");
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
  data_ts   = (message_data_ts_t *)data_ptr;
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
  data_ts   = (message_data_ts_t *)data_ptr;
  data->jowl_action = ini_jowl_action;
  add_fieldinfo(IFT_ENUM, "jowl_action", 1, &data->jowl_action, "jowl_t");
}
/** Constructor */
FacialExpressionInterface::MoveJowlMessage::MoveJowlMessage() : Message("MoveJowlMessage")
{
  data_size = sizeof(MoveJowlMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveJowlMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_ENUM, "jowl_action", 1, &data->jowl_action, "jowl_t");
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
  data_ts   = (message_data_ts_t *)data_ptr;
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
  data_ts   = (message_data_ts_t *)data_ptr;
  data->mouth_action = ini_mouth_action;
  add_fieldinfo(IFT_ENUM, "mouth_action", 1, &data->mouth_action, "mouth_t");
}
/** Constructor */
FacialExpressionInterface::MoveMouthMessage::MoveMouthMessage() : Message("MoveMouthMessage")
{
  data_size = sizeof(MoveMouthMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveMouthMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_ENUM, "mouth_action", 1, &data->mouth_action, "mouth_t");
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
  data_ts   = (message_data_ts_t *)data_ptr;
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
