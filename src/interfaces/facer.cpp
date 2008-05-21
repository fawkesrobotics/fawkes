
/***************************************************************************
 *  facer.cpp - Fawkes BlackBoard Interface - FacerInterface
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

#include <interfaces/facer.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class FacerInterface <interfaces/facer.h>
 * FacerInterface Fawkes BlackBoard Interface.
 * 
      The interface provides access to the face recognition plugin (facer).
      It provides basic status information about facer and allows for setting a specific
      mode and access the resolut.
      calling skills via messages. It can also be used to manually restart
      the Lua interpreter if something is wedged.
    
 */



/** Constructor */
FacerInterface::FacerInterface() : Interface()
{
  data_size = sizeof(FacerInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (FacerInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  unsigned char tmp_hash[] = {0x98, 0xd0, 0xac, 0x73, 0xe, 0x11, 0x3f, 00, 0x3e, 0xe7, 0x6b, 0x7a, 0x4e, 0xdc, 0xd2, 0xd0};
  set_hash(tmp_hash);
}

/** Destructor */
FacerInterface::~FacerInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get opmode value.
 * Current opmode.
 * @return opmode value
 */
FacerInterface::if_facer_opmode_t
FacerInterface::opmode()
{
  return data->opmode;
}

/** Get maximum length of opmode value.
 * @return length of opmode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_opmode() const
{
  return 1;
}

/** Set opmode value.
 * Current opmode.
 * @param new_opmode new opmode value
 */
void
FacerInterface::set_opmode(const if_facer_opmode_t new_opmode)
{
  data->opmode = new_opmode;
}

/** Get face_label value.
 * Label of the recognized face
 * @return face_label value
 */
char *
FacerInterface::face_label()
{
  return data->face_label;
}

/** Get maximum length of face_label value.
 * @return length of face_label value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_face_label() const
{
  return 64;
}

/** Set face_label value.
 * Label of the recognized face
 * @param new_face_label new face_label value
 */
void
FacerInterface::set_face_label(const char * new_face_label)
{
  strncpy(data->face_label, new_face_label, sizeof(data->face_label));
}

/* =========== message create =========== */
Message *
FacerInterface::create_message(const char *type) const
{
  if ( strncmp("LearnFaceMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new LearnFaceMessage();
  } else if ( strncmp("SetOpmodeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetOpmodeMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/* =========== messages =========== */
/** @class FacerInterface::LearnFaceMessage interfaces/facer.h
 * LearnFaceMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_face_label initial value for face_label
 */
FacerInterface::LearnFaceMessage::LearnFaceMessage(const char * ini_face_label) : Message("LearnFaceMessage")
{
  data_size = sizeof(LearnFaceMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (LearnFaceMessage_data_t *)data_ptr;
  strncpy(data->face_label, ini_face_label, 64);
}
/** Constructor */
FacerInterface::LearnFaceMessage::LearnFaceMessage() : Message("LearnFaceMessage")
{
  data_size = sizeof(LearnFaceMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (LearnFaceMessage_data_t *)data_ptr;
}

/** Destructor */
FacerInterface::LearnFaceMessage::~LearnFaceMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
FacerInterface::LearnFaceMessage::LearnFaceMessage(const LearnFaceMessage *m) : Message("LearnFaceMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (LearnFaceMessage_data_t *)data_ptr;
}

/* Methods */
/** Get face_label value.
 * Label of the recognized face
 * @return face_label value
 */
char *
FacerInterface::LearnFaceMessage::face_label()
{
  return data->face_label;
}

/** Get maximum length of face_label value.
 * @return length of face_label value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::LearnFaceMessage::maxlenof_face_label() const
{
  return 64;
}

/** Set face_label value.
 * Label of the recognized face
 * @param new_face_label new face_label value
 */
void
FacerInterface::LearnFaceMessage::set_face_label(const char * new_face_label)
{
  strncpy(data->face_label, new_face_label, sizeof(data->face_label));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
FacerInterface::LearnFaceMessage::clone() const
{
  return new FacerInterface::LearnFaceMessage(this);
}
/** @class FacerInterface::SetOpmodeMessage interfaces/facer.h
 * SetOpmodeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_opmode initial value for opmode
 */
FacerInterface::SetOpmodeMessage::SetOpmodeMessage(const if_facer_opmode_t ini_opmode) : Message("SetOpmodeMessage")
{
  data_size = sizeof(SetOpmodeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetOpmodeMessage_data_t *)data_ptr;
  data->opmode = ini_opmode;
}
/** Constructor */
FacerInterface::SetOpmodeMessage::SetOpmodeMessage() : Message("SetOpmodeMessage")
{
  data_size = sizeof(SetOpmodeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetOpmodeMessage_data_t *)data_ptr;
}

/** Destructor */
FacerInterface::SetOpmodeMessage::~SetOpmodeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
FacerInterface::SetOpmodeMessage::SetOpmodeMessage(const SetOpmodeMessage *m) : Message("SetOpmodeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetOpmodeMessage_data_t *)data_ptr;
}

/* Methods */
/** Get opmode value.
 * Current opmode.
 * @return opmode value
 */
FacerInterface::if_facer_opmode_t
FacerInterface::SetOpmodeMessage::opmode()
{
  return data->opmode;
}

/** Get maximum length of opmode value.
 * @return length of opmode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::SetOpmodeMessage::maxlenof_opmode() const
{
  return 1;
}

/** Set opmode value.
 * Current opmode.
 * @param new_opmode new opmode value
 */
void
FacerInterface::SetOpmodeMessage::set_opmode(const if_facer_opmode_t new_opmode)
{
  data->opmode = new_opmode;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
FacerInterface::SetOpmodeMessage::clone() const
{
  return new FacerInterface::SetOpmodeMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
FacerInterface::message_valid(const Message *message) const
{
  const LearnFaceMessage *m0 = dynamic_cast<const LearnFaceMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SetOpmodeMessage *m1 = dynamic_cast<const SetOpmodeMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(FacerInterface)
/// @endcond


} // end namespace fawkes
