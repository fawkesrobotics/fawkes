
/***************************************************************************
 *  OpenRaveInterface.cpp - Fawkes BlackBoard Interface - OpenRaveInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2011  Bahram Maleki-Fard
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

#include <interfaces/OpenRaveInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class OpenRaveInterface <interfaces/OpenRaveInterface.h>
 * OpenRaveInterface Fawkes BlackBoard Interface.
 * 
      Interface providing access to OpenRAVE functionality
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
OpenRaveInterface::OpenRaveInterface() : Interface()
{
  data_size = sizeof(OpenRaveInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (OpenRaveInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_UINT32, "error_code", 1, &data->error_code);
  add_fieldinfo(IFT_BOOL, "success", 1, &data->success);
  add_messageinfo("StartViewerMessage");
  add_messageinfo("AddObjectMessage");
  add_messageinfo("DeleteObjectMessage");
  add_messageinfo("DeleteAllObjectsMessage");
  add_messageinfo("AttachObjectMessage");
  add_messageinfo("ReleaseObjectMessage");
  add_messageinfo("ReleaseAllObjectsMessage");
  add_messageinfo("MoveObjectMessage");
  add_messageinfo("RotateObjectQuatMessage");
  add_messageinfo("RotateObjectMessage");
  add_messageinfo("RenameObjectMessage");
  unsigned char tmp_hash[] = {0xac, 0x95, 0xde, 0xc, 0xea, 0xa4, 0x97, 0x56, 0x5c, 0x46, 0x11, 0x5b, 0xf7, 0x60, 0x41, 0xb};
  set_hash(tmp_hash);
}

/** Destructor */
OpenRaveInterface::~OpenRaveInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @return msgid value
 */
uint32_t
OpenRaveInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @param new_msgid new msgid value
 */
void
OpenRaveInterface::set_msgid(const uint32_t new_msgid)
{
  data->msgid = new_msgid;
  data_changed = true;
}

/** Get final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @return final value
 */
bool
OpenRaveInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @param new_final new final value
 */
void
OpenRaveInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/** Get error_code value.
 * Failure code set if
      final is true. 0 if no error occured, an error code from ERROR_*
      constants otherwise (or a bit-wise combination).
 * @return error_code value
 */
uint32_t
OpenRaveInterface::error_code() const
{
  return data->error_code;
}

/** Get maximum length of error_code value.
 * @return length of error_code value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::maxlenof_error_code() const
{
  return 1;
}

/** Set error_code value.
 * Failure code set if
      final is true. 0 if no error occured, an error code from ERROR_*
      constants otherwise (or a bit-wise combination).
 * @param new_error_code new error_code value
 */
void
OpenRaveInterface::set_error_code(const uint32_t new_error_code)
{
  data->error_code = new_error_code;
  data_changed = true;
}

/** Get success value.
 * True, if last command was successful. False otherwise
 * @return success value
 */
bool
OpenRaveInterface::is_success() const
{
  return data->success;
}

/** Get maximum length of success value.
 * @return length of success value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::maxlenof_success() const
{
  return 1;
}

/** Set success value.
 * True, if last command was successful. False otherwise
 * @param new_success new success value
 */
void
OpenRaveInterface::set_success(const bool new_success)
{
  data->success = new_success;
  data_changed = true;
}

/* =========== message create =========== */
Message *
OpenRaveInterface::create_message(const char *type) const
{
  if ( strncmp("StartViewerMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StartViewerMessage();
  } else if ( strncmp("AddObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddObjectMessage();
  } else if ( strncmp("DeleteObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new DeleteObjectMessage();
  } else if ( strncmp("DeleteAllObjectsMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new DeleteAllObjectsMessage();
  } else if ( strncmp("AttachObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AttachObjectMessage();
  } else if ( strncmp("ReleaseObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ReleaseObjectMessage();
  } else if ( strncmp("ReleaseAllObjectsMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ReleaseAllObjectsMessage();
  } else if ( strncmp("MoveObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new MoveObjectMessage();
  } else if ( strncmp("RotateObjectQuatMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RotateObjectQuatMessage();
  } else if ( strncmp("RotateObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RotateObjectMessage();
  } else if ( strncmp("RenameObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RenameObjectMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
OpenRaveInterface::copy_values(const Interface *other)
{
  const OpenRaveInterface *oi = dynamic_cast<const OpenRaveInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(OpenRaveInterface_data_t));
}

const char *
OpenRaveInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class OpenRaveInterface::StartViewerMessage <interfaces/OpenRaveInterface.h>
 * StartViewerMessage Fawkes BlackBoard Interface Message.
 * 
    Start the qtcoin viewer, showing the current OpenRAVE environment.
  
 */


/** Constructor */
OpenRaveInterface::StartViewerMessage::StartViewerMessage() : Message("StartViewerMessage")
{
  data_size = sizeof(StartViewerMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StartViewerMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
OpenRaveInterface::StartViewerMessage::~StartViewerMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRaveInterface::StartViewerMessage::StartViewerMessage(const StartViewerMessage *m) : Message("StartViewerMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StartViewerMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRaveInterface::StartViewerMessage::clone() const
{
  return new OpenRaveInterface::StartViewerMessage(this);
}
/** @class OpenRaveInterface::AddObjectMessage <interfaces/OpenRaveInterface.h>
 * AddObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 * @param ini_path initial value for path
 */
OpenRaveInterface::AddObjectMessage::AddObjectMessage(const char * ini_name, const char * ini_path) : Message("AddObjectMessage")
{
  data_size = sizeof(AddObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30-1);
  data->name[30-1] = 0;
  strncpy(data->path, ini_path, 1024-1);
  data->path[1024-1] = 0;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_STRING, "path", 1024, data->path);
}
/** Constructor */
OpenRaveInterface::AddObjectMessage::AddObjectMessage() : Message("AddObjectMessage")
{
  data_size = sizeof(AddObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_STRING, "path", 1024, data->path);
}

/** Destructor */
OpenRaveInterface::AddObjectMessage::~AddObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRaveInterface::AddObjectMessage::AddObjectMessage(const AddObjectMessage *m) : Message("AddObjectMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AddObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get name value.
 * Name of object
 * @return name value
 */
char *
OpenRaveInterface::AddObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::AddObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRaveInterface::AddObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name)-1);
  data->name[sizeof(data->name)-1] = 0;
}

/** Get path value.
 * Path to object xml file
 * @return path value
 */
char *
OpenRaveInterface::AddObjectMessage::path() const
{
  return data->path;
}

/** Get maximum length of path value.
 * @return length of path value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::AddObjectMessage::maxlenof_path() const
{
  return 1024;
}

/** Set path value.
 * Path to object xml file
 * @param new_path new path value
 */
void
OpenRaveInterface::AddObjectMessage::set_path(const char * new_path)
{
  strncpy(data->path, new_path, sizeof(data->path)-1);
  data->path[sizeof(data->path)-1] = 0;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRaveInterface::AddObjectMessage::clone() const
{
  return new OpenRaveInterface::AddObjectMessage(this);
}
/** @class OpenRaveInterface::DeleteObjectMessage <interfaces/OpenRaveInterface.h>
 * DeleteObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 */
OpenRaveInterface::DeleteObjectMessage::DeleteObjectMessage(const char * ini_name) : Message("DeleteObjectMessage")
{
  data_size = sizeof(DeleteObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DeleteObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30-1);
  data->name[30-1] = 0;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
}
/** Constructor */
OpenRaveInterface::DeleteObjectMessage::DeleteObjectMessage() : Message("DeleteObjectMessage")
{
  data_size = sizeof(DeleteObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DeleteObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
}

/** Destructor */
OpenRaveInterface::DeleteObjectMessage::~DeleteObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRaveInterface::DeleteObjectMessage::DeleteObjectMessage(const DeleteObjectMessage *m) : Message("DeleteObjectMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (DeleteObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get name value.
 * Name of object
 * @return name value
 */
char *
OpenRaveInterface::DeleteObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::DeleteObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRaveInterface::DeleteObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name)-1);
  data->name[sizeof(data->name)-1] = 0;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRaveInterface::DeleteObjectMessage::clone() const
{
  return new OpenRaveInterface::DeleteObjectMessage(this);
}
/** @class OpenRaveInterface::DeleteAllObjectsMessage <interfaces/OpenRaveInterface.h>
 * DeleteAllObjectsMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
OpenRaveInterface::DeleteAllObjectsMessage::DeleteAllObjectsMessage() : Message("DeleteAllObjectsMessage")
{
  data_size = sizeof(DeleteAllObjectsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DeleteAllObjectsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
OpenRaveInterface::DeleteAllObjectsMessage::~DeleteAllObjectsMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRaveInterface::DeleteAllObjectsMessage::DeleteAllObjectsMessage(const DeleteAllObjectsMessage *m) : Message("DeleteAllObjectsMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (DeleteAllObjectsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRaveInterface::DeleteAllObjectsMessage::clone() const
{
  return new OpenRaveInterface::DeleteAllObjectsMessage(this);
}
/** @class OpenRaveInterface::AttachObjectMessage <interfaces/OpenRaveInterface.h>
 * AttachObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 * @param ini_manip_name initial value for manip_name
 */
OpenRaveInterface::AttachObjectMessage::AttachObjectMessage(const char * ini_name, const char * ini_manip_name) : Message("AttachObjectMessage")
{
  data_size = sizeof(AttachObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AttachObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30-1);
  data->name[30-1] = 0;
  strncpy(data->manip_name, ini_manip_name, 30-1);
  data->manip_name[30-1] = 0;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_STRING, "manip_name", 30, data->manip_name);
}
/** Constructor */
OpenRaveInterface::AttachObjectMessage::AttachObjectMessage() : Message("AttachObjectMessage")
{
  data_size = sizeof(AttachObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AttachObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_STRING, "manip_name", 30, data->manip_name);
}

/** Destructor */
OpenRaveInterface::AttachObjectMessage::~AttachObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRaveInterface::AttachObjectMessage::AttachObjectMessage(const AttachObjectMessage *m) : Message("AttachObjectMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AttachObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get name value.
 * Name of object
 * @return name value
 */
char *
OpenRaveInterface::AttachObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::AttachObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRaveInterface::AttachObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name)-1);
  data->name[sizeof(data->name)-1] = 0;
}

/** Get manip_name value.
 * Name of manipulator
 * @return manip_name value
 */
char *
OpenRaveInterface::AttachObjectMessage::manip_name() const
{
  return data->manip_name;
}

/** Get maximum length of manip_name value.
 * @return length of manip_name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::AttachObjectMessage::maxlenof_manip_name() const
{
  return 30;
}

/** Set manip_name value.
 * Name of manipulator
 * @param new_manip_name new manip_name value
 */
void
OpenRaveInterface::AttachObjectMessage::set_manip_name(const char * new_manip_name)
{
  strncpy(data->manip_name, new_manip_name, sizeof(data->manip_name)-1);
  data->manip_name[sizeof(data->manip_name)-1] = 0;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRaveInterface::AttachObjectMessage::clone() const
{
  return new OpenRaveInterface::AttachObjectMessage(this);
}
/** @class OpenRaveInterface::ReleaseObjectMessage <interfaces/OpenRaveInterface.h>
 * ReleaseObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 */
OpenRaveInterface::ReleaseObjectMessage::ReleaseObjectMessage(const char * ini_name) : Message("ReleaseObjectMessage")
{
  data_size = sizeof(ReleaseObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ReleaseObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30-1);
  data->name[30-1] = 0;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
}
/** Constructor */
OpenRaveInterface::ReleaseObjectMessage::ReleaseObjectMessage() : Message("ReleaseObjectMessage")
{
  data_size = sizeof(ReleaseObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ReleaseObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
}

/** Destructor */
OpenRaveInterface::ReleaseObjectMessage::~ReleaseObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRaveInterface::ReleaseObjectMessage::ReleaseObjectMessage(const ReleaseObjectMessage *m) : Message("ReleaseObjectMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ReleaseObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get name value.
 * Name of object
 * @return name value
 */
char *
OpenRaveInterface::ReleaseObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::ReleaseObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRaveInterface::ReleaseObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name)-1);
  data->name[sizeof(data->name)-1] = 0;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRaveInterface::ReleaseObjectMessage::clone() const
{
  return new OpenRaveInterface::ReleaseObjectMessage(this);
}
/** @class OpenRaveInterface::ReleaseAllObjectsMessage <interfaces/OpenRaveInterface.h>
 * ReleaseAllObjectsMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
OpenRaveInterface::ReleaseAllObjectsMessage::ReleaseAllObjectsMessage() : Message("ReleaseAllObjectsMessage")
{
  data_size = sizeof(ReleaseAllObjectsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ReleaseAllObjectsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
OpenRaveInterface::ReleaseAllObjectsMessage::~ReleaseAllObjectsMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRaveInterface::ReleaseAllObjectsMessage::ReleaseAllObjectsMessage(const ReleaseAllObjectsMessage *m) : Message("ReleaseAllObjectsMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ReleaseAllObjectsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRaveInterface::ReleaseAllObjectsMessage::clone() const
{
  return new OpenRaveInterface::ReleaseAllObjectsMessage(this);
}
/** @class OpenRaveInterface::MoveObjectMessage <interfaces/OpenRaveInterface.h>
 * MoveObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_z initial value for z
 */
OpenRaveInterface::MoveObjectMessage::MoveObjectMessage(const char * ini_name, const float ini_x, const float ini_y, const float ini_z) : Message("MoveObjectMessage")
{
  data_size = sizeof(MoveObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30-1);
  data->name[30-1] = 0;
  data->x = ini_x;
  data->y = ini_y;
  data->z = ini_z;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
}
/** Constructor */
OpenRaveInterface::MoveObjectMessage::MoveObjectMessage() : Message("MoveObjectMessage")
{
  data_size = sizeof(MoveObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
}

/** Destructor */
OpenRaveInterface::MoveObjectMessage::~MoveObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRaveInterface::MoveObjectMessage::MoveObjectMessage(const MoveObjectMessage *m) : Message("MoveObjectMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (MoveObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get name value.
 * Name of object
 * @return name value
 */
char *
OpenRaveInterface::MoveObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::MoveObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRaveInterface::MoveObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name)-1);
  data->name[sizeof(data->name)-1] = 0;
}

/** Get x value.
 * x position of object (meters)
 * @return x value
 */
float
OpenRaveInterface::MoveObjectMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::MoveObjectMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * x position of object (meters)
 * @param new_x new x value
 */
void
OpenRaveInterface::MoveObjectMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * y position of object (meters)
 * @return y value
 */
float
OpenRaveInterface::MoveObjectMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::MoveObjectMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * y position of object (meters)
 * @param new_y new y value
 */
void
OpenRaveInterface::MoveObjectMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get z value.
 * z position of object (meters)
 * @return z value
 */
float
OpenRaveInterface::MoveObjectMessage::z() const
{
  return data->z;
}

/** Get maximum length of z value.
 * @return length of z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::MoveObjectMessage::maxlenof_z() const
{
  return 1;
}

/** Set z value.
 * z position of object (meters)
 * @param new_z new z value
 */
void
OpenRaveInterface::MoveObjectMessage::set_z(const float new_z)
{
  data->z = new_z;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRaveInterface::MoveObjectMessage::clone() const
{
  return new OpenRaveInterface::MoveObjectMessage(this);
}
/** @class OpenRaveInterface::RotateObjectQuatMessage <interfaces/OpenRaveInterface.h>
 * RotateObjectQuatMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_z initial value for z
 * @param ini_w initial value for w
 */
OpenRaveInterface::RotateObjectQuatMessage::RotateObjectQuatMessage(const char * ini_name, const float ini_x, const float ini_y, const float ini_z, const float ini_w) : Message("RotateObjectQuatMessage")
{
  data_size = sizeof(RotateObjectQuatMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RotateObjectQuatMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30-1);
  data->name[30-1] = 0;
  data->x = ini_x;
  data->y = ini_y;
  data->z = ini_z;
  data->w = ini_w;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
  add_fieldinfo(IFT_FLOAT, "w", 1, &data->w);
}
/** Constructor */
OpenRaveInterface::RotateObjectQuatMessage::RotateObjectQuatMessage() : Message("RotateObjectQuatMessage")
{
  data_size = sizeof(RotateObjectQuatMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RotateObjectQuatMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
  add_fieldinfo(IFT_FLOAT, "w", 1, &data->w);
}

/** Destructor */
OpenRaveInterface::RotateObjectQuatMessage::~RotateObjectQuatMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRaveInterface::RotateObjectQuatMessage::RotateObjectQuatMessage(const RotateObjectQuatMessage *m) : Message("RotateObjectQuatMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (RotateObjectQuatMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get name value.
 * Name of object
 * @return name value
 */
char *
OpenRaveInterface::RotateObjectQuatMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::RotateObjectQuatMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRaveInterface::RotateObjectQuatMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name)-1);
  data->name[sizeof(data->name)-1] = 0;
}

/** Get x value.
 * x value of quaternion
 * @return x value
 */
float
OpenRaveInterface::RotateObjectQuatMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::RotateObjectQuatMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * x value of quaternion
 * @param new_x new x value
 */
void
OpenRaveInterface::RotateObjectQuatMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * y value of quaternion
 * @return y value
 */
float
OpenRaveInterface::RotateObjectQuatMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::RotateObjectQuatMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * y value of quaternion
 * @param new_y new y value
 */
void
OpenRaveInterface::RotateObjectQuatMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get z value.
 * z value of quaternion
 * @return z value
 */
float
OpenRaveInterface::RotateObjectQuatMessage::z() const
{
  return data->z;
}

/** Get maximum length of z value.
 * @return length of z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::RotateObjectQuatMessage::maxlenof_z() const
{
  return 1;
}

/** Set z value.
 * z value of quaternion
 * @param new_z new z value
 */
void
OpenRaveInterface::RotateObjectQuatMessage::set_z(const float new_z)
{
  data->z = new_z;
}

/** Get w value.
 * w value of quaternion
 * @return w value
 */
float
OpenRaveInterface::RotateObjectQuatMessage::w() const
{
  return data->w;
}

/** Get maximum length of w value.
 * @return length of w value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::RotateObjectQuatMessage::maxlenof_w() const
{
  return 1;
}

/** Set w value.
 * w value of quaternion
 * @param new_w new w value
 */
void
OpenRaveInterface::RotateObjectQuatMessage::set_w(const float new_w)
{
  data->w = new_w;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRaveInterface::RotateObjectQuatMessage::clone() const
{
  return new OpenRaveInterface::RotateObjectQuatMessage(this);
}
/** @class OpenRaveInterface::RotateObjectMessage <interfaces/OpenRaveInterface.h>
 * RotateObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_z initial value for z
 */
OpenRaveInterface::RotateObjectMessage::RotateObjectMessage(const char * ini_name, const float ini_x, const float ini_y, const float ini_z) : Message("RotateObjectMessage")
{
  data_size = sizeof(RotateObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RotateObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30-1);
  data->name[30-1] = 0;
  data->x = ini_x;
  data->y = ini_y;
  data->z = ini_z;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
}
/** Constructor */
OpenRaveInterface::RotateObjectMessage::RotateObjectMessage() : Message("RotateObjectMessage")
{
  data_size = sizeof(RotateObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RotateObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
}

/** Destructor */
OpenRaveInterface::RotateObjectMessage::~RotateObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRaveInterface::RotateObjectMessage::RotateObjectMessage(const RotateObjectMessage *m) : Message("RotateObjectMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (RotateObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get name value.
 * Name of object
 * @return name value
 */
char *
OpenRaveInterface::RotateObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::RotateObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRaveInterface::RotateObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name)-1);
  data->name[sizeof(data->name)-1] = 0;
}

/** Get x value.
 * x-axis rotation of object (rad)
 * @return x value
 */
float
OpenRaveInterface::RotateObjectMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::RotateObjectMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * x-axis rotation of object (rad)
 * @param new_x new x value
 */
void
OpenRaveInterface::RotateObjectMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * y-axis rotation of object (rad)
 * @return y value
 */
float
OpenRaveInterface::RotateObjectMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::RotateObjectMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * y-axis rotation of object (rad)
 * @param new_y new y value
 */
void
OpenRaveInterface::RotateObjectMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get z value.
 * z-axis rotation of object (rad)
 * @return z value
 */
float
OpenRaveInterface::RotateObjectMessage::z() const
{
  return data->z;
}

/** Get maximum length of z value.
 * @return length of z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::RotateObjectMessage::maxlenof_z() const
{
  return 1;
}

/** Set z value.
 * z-axis rotation of object (rad)
 * @param new_z new z value
 */
void
OpenRaveInterface::RotateObjectMessage::set_z(const float new_z)
{
  data->z = new_z;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRaveInterface::RotateObjectMessage::clone() const
{
  return new OpenRaveInterface::RotateObjectMessage(this);
}
/** @class OpenRaveInterface::RenameObjectMessage <interfaces/OpenRaveInterface.h>
 * RenameObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 * @param ini_newName initial value for newName
 */
OpenRaveInterface::RenameObjectMessage::RenameObjectMessage(const char * ini_name, const char * ini_newName) : Message("RenameObjectMessage")
{
  data_size = sizeof(RenameObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RenameObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30-1);
  data->name[30-1] = 0;
  strncpy(data->newName, ini_newName, 30-1);
  data->newName[30-1] = 0;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_STRING, "newName", 30, data->newName);
}
/** Constructor */
OpenRaveInterface::RenameObjectMessage::RenameObjectMessage() : Message("RenameObjectMessage")
{
  data_size = sizeof(RenameObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RenameObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_STRING, "newName", 30, data->newName);
}

/** Destructor */
OpenRaveInterface::RenameObjectMessage::~RenameObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRaveInterface::RenameObjectMessage::RenameObjectMessage(const RenameObjectMessage *m) : Message("RenameObjectMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (RenameObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get name value.
 * Name of object
 * @return name value
 */
char *
OpenRaveInterface::RenameObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::RenameObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRaveInterface::RenameObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name)-1);
  data->name[sizeof(data->name)-1] = 0;
}

/** Get newName value.
 * New name of object
 * @return newName value
 */
char *
OpenRaveInterface::RenameObjectMessage::newName() const
{
  return data->newName;
}

/** Get maximum length of newName value.
 * @return length of newName value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRaveInterface::RenameObjectMessage::maxlenof_newName() const
{
  return 30;
}

/** Set newName value.
 * New name of object
 * @param new_newName new newName value
 */
void
OpenRaveInterface::RenameObjectMessage::set_newName(const char * new_newName)
{
  strncpy(data->newName, new_newName, sizeof(data->newName)-1);
  data->newName[sizeof(data->newName)-1] = 0;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRaveInterface::RenameObjectMessage::clone() const
{
  return new OpenRaveInterface::RenameObjectMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
OpenRaveInterface::message_valid(const Message *message) const
{
  const StartViewerMessage *m0 = dynamic_cast<const StartViewerMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const AddObjectMessage *m1 = dynamic_cast<const AddObjectMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const DeleteObjectMessage *m2 = dynamic_cast<const DeleteObjectMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const DeleteAllObjectsMessage *m3 = dynamic_cast<const DeleteAllObjectsMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const AttachObjectMessage *m4 = dynamic_cast<const AttachObjectMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const ReleaseObjectMessage *m5 = dynamic_cast<const ReleaseObjectMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const ReleaseAllObjectsMessage *m6 = dynamic_cast<const ReleaseAllObjectsMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const MoveObjectMessage *m7 = dynamic_cast<const MoveObjectMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const RotateObjectQuatMessage *m8 = dynamic_cast<const RotateObjectQuatMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  const RotateObjectMessage *m9 = dynamic_cast<const RotateObjectMessage *>(message);
  if ( m9 != NULL ) {
    return true;
  }
  const RenameObjectMessage *m10 = dynamic_cast<const RenameObjectMessage *>(message);
  if ( m10 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(OpenRaveInterface)
/// @endcond


} // end namespace fawkes
