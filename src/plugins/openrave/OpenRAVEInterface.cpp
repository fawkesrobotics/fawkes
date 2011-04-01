
/***************************************************************************
 *  OpenRAVEInterface.cpp - Fawkes BlackBoard Interface - OpenRAVEInterface
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

#include <interfaces/OpenRAVEInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class OpenRAVEInterface <interfaces/OpenRAVEInterface.h>
 * OpenRAVEInterface Fawkes BlackBoard Interface.
 * 
      Interface providing access to OpenRAVE functionality
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
OpenRAVEInterface::OpenRAVEInterface() : Interface()
{
  data_size = sizeof(OpenRAVEInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (OpenRAVEInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_UINT32, "error_code", 1, &data->error_code);
  add_fieldinfo(IFT_BOOL, "success", 1, &data->success);
  add_messageinfo("AddObjectMessage");
  add_messageinfo("DeleteObjectMessage");
  add_messageinfo("AttachObjectMessage");
  add_messageinfo("ReleaseObjectMessage");
  add_messageinfo("ReleaseAllObjectsMessage");
  add_messageinfo("MoveObjectMessage");
  add_messageinfo("RotateObjectMessage");
  add_messageinfo("RenameObjectMessage");
  unsigned char tmp_hash[] = {0x5e, 0x34, 0xb9, 0xd9, 0x92, 0x45, 0x44, 0x88, 0xe2, 0xde, 0x4e, 0x92, 0xff, 0x37, 0x4a, 0x14};
  set_hash(tmp_hash);
}

/** Destructor */
OpenRAVEInterface::~OpenRAVEInterface()
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
OpenRAVEInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @param new_msgid new msgid value
 */
void
OpenRAVEInterface::set_msgid(const uint32_t new_msgid)
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
OpenRAVEInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @param new_final new final value
 */
void
OpenRAVEInterface::set_final(const bool new_final)
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
OpenRAVEInterface::error_code() const
{
  return data->error_code;
}

/** Get maximum length of error_code value.
 * @return length of error_code value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::maxlenof_error_code() const
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
OpenRAVEInterface::set_error_code(const uint32_t new_error_code)
{
  data->error_code = new_error_code;
  data_changed = true;
}

/** Get success value.
 * True, if last command was successful. False otherwise
 * @return success value
 */
bool
OpenRAVEInterface::is_success() const
{
  return data->success;
}

/** Get maximum length of success value.
 * @return length of success value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::maxlenof_success() const
{
  return 1;
}

/** Set success value.
 * True, if last command was successful. False otherwise
 * @param new_success new success value
 */
void
OpenRAVEInterface::set_success(const bool new_success)
{
  data->success = new_success;
  data_changed = true;
}

/* =========== message create =========== */
Message *
OpenRAVEInterface::create_message(const char *type) const
{
  if ( strncmp("AddObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddObjectMessage();
  } else if ( strncmp("DeleteObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new DeleteObjectMessage();
  } else if ( strncmp("AttachObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AttachObjectMessage();
  } else if ( strncmp("ReleaseObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ReleaseObjectMessage();
  } else if ( strncmp("ReleaseAllObjectsMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ReleaseAllObjectsMessage();
  } else if ( strncmp("MoveObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new MoveObjectMessage();
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
OpenRAVEInterface::copy_values(const Interface *other)
{
  const OpenRAVEInterface *oi = dynamic_cast<const OpenRAVEInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(OpenRAVEInterface_data_t));
}

const char *
OpenRAVEInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class OpenRAVEInterface::AddObjectMessage <interfaces/OpenRAVEInterface.h>
 * AddObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 * @param ini_path initial value for path
 */
OpenRAVEInterface::AddObjectMessage::AddObjectMessage(const char * ini_name, const char * ini_path) : Message("AddObjectMessage")
{
  data_size = sizeof(AddObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30);
  strncpy(data->path, ini_path, 1024);
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_STRING, "path", 1024, data->path);
}
/** Constructor */
OpenRAVEInterface::AddObjectMessage::AddObjectMessage() : Message("AddObjectMessage")
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
OpenRAVEInterface::AddObjectMessage::~AddObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRAVEInterface::AddObjectMessage::AddObjectMessage(const AddObjectMessage *m) : Message("AddObjectMessage")
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
OpenRAVEInterface::AddObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::AddObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRAVEInterface::AddObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name));
}

/** Get path value.
 * Path to object xml file
 * @return path value
 */
char *
OpenRAVEInterface::AddObjectMessage::path() const
{
  return data->path;
}

/** Get maximum length of path value.
 * @return length of path value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::AddObjectMessage::maxlenof_path() const
{
  return 1024;
}

/** Set path value.
 * Path to object xml file
 * @param new_path new path value
 */
void
OpenRAVEInterface::AddObjectMessage::set_path(const char * new_path)
{
  strncpy(data->path, new_path, sizeof(data->path));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRAVEInterface::AddObjectMessage::clone() const
{
  return new OpenRAVEInterface::AddObjectMessage(this);
}
/** @class OpenRAVEInterface::DeleteObjectMessage <interfaces/OpenRAVEInterface.h>
 * DeleteObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 */
OpenRAVEInterface::DeleteObjectMessage::DeleteObjectMessage(const char * ini_name) : Message("DeleteObjectMessage")
{
  data_size = sizeof(DeleteObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DeleteObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30);
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
}
/** Constructor */
OpenRAVEInterface::DeleteObjectMessage::DeleteObjectMessage() : Message("DeleteObjectMessage")
{
  data_size = sizeof(DeleteObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DeleteObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
}

/** Destructor */
OpenRAVEInterface::DeleteObjectMessage::~DeleteObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRAVEInterface::DeleteObjectMessage::DeleteObjectMessage(const DeleteObjectMessage *m) : Message("DeleteObjectMessage")
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
OpenRAVEInterface::DeleteObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::DeleteObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRAVEInterface::DeleteObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRAVEInterface::DeleteObjectMessage::clone() const
{
  return new OpenRAVEInterface::DeleteObjectMessage(this);
}
/** @class OpenRAVEInterface::AttachObjectMessage <interfaces/OpenRAVEInterface.h>
 * AttachObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 */
OpenRAVEInterface::AttachObjectMessage::AttachObjectMessage(const char * ini_name) : Message("AttachObjectMessage")
{
  data_size = sizeof(AttachObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AttachObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30);
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
}
/** Constructor */
OpenRAVEInterface::AttachObjectMessage::AttachObjectMessage() : Message("AttachObjectMessage")
{
  data_size = sizeof(AttachObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AttachObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
}

/** Destructor */
OpenRAVEInterface::AttachObjectMessage::~AttachObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRAVEInterface::AttachObjectMessage::AttachObjectMessage(const AttachObjectMessage *m) : Message("AttachObjectMessage")
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
OpenRAVEInterface::AttachObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::AttachObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRAVEInterface::AttachObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRAVEInterface::AttachObjectMessage::clone() const
{
  return new OpenRAVEInterface::AttachObjectMessage(this);
}
/** @class OpenRAVEInterface::ReleaseObjectMessage <interfaces/OpenRAVEInterface.h>
 * ReleaseObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 */
OpenRAVEInterface::ReleaseObjectMessage::ReleaseObjectMessage(const char * ini_name) : Message("ReleaseObjectMessage")
{
  data_size = sizeof(ReleaseObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ReleaseObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30);
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
}
/** Constructor */
OpenRAVEInterface::ReleaseObjectMessage::ReleaseObjectMessage() : Message("ReleaseObjectMessage")
{
  data_size = sizeof(ReleaseObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ReleaseObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
}

/** Destructor */
OpenRAVEInterface::ReleaseObjectMessage::~ReleaseObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRAVEInterface::ReleaseObjectMessage::ReleaseObjectMessage(const ReleaseObjectMessage *m) : Message("ReleaseObjectMessage")
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
OpenRAVEInterface::ReleaseObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::ReleaseObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRAVEInterface::ReleaseObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRAVEInterface::ReleaseObjectMessage::clone() const
{
  return new OpenRAVEInterface::ReleaseObjectMessage(this);
}
/** @class OpenRAVEInterface::ReleaseAllObjectsMessage <interfaces/OpenRAVEInterface.h>
 * ReleaseAllObjectsMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
OpenRAVEInterface::ReleaseAllObjectsMessage::ReleaseAllObjectsMessage() : Message("ReleaseAllObjectsMessage")
{
  data_size = sizeof(ReleaseAllObjectsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ReleaseAllObjectsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
OpenRAVEInterface::ReleaseAllObjectsMessage::~ReleaseAllObjectsMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRAVEInterface::ReleaseAllObjectsMessage::ReleaseAllObjectsMessage(const ReleaseAllObjectsMessage *m) : Message("ReleaseAllObjectsMessage")
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
OpenRAVEInterface::ReleaseAllObjectsMessage::clone() const
{
  return new OpenRAVEInterface::ReleaseAllObjectsMessage(this);
}
/** @class OpenRAVEInterface::MoveObjectMessage <interfaces/OpenRAVEInterface.h>
 * MoveObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_z initial value for z
 */
OpenRAVEInterface::MoveObjectMessage::MoveObjectMessage(const char * ini_name, const float ini_x, const float ini_y, const float ini_z) : Message("MoveObjectMessage")
{
  data_size = sizeof(MoveObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30);
  data->x = ini_x;
  data->y = ini_y;
  data->z = ini_z;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
}
/** Constructor */
OpenRAVEInterface::MoveObjectMessage::MoveObjectMessage() : Message("MoveObjectMessage")
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
OpenRAVEInterface::MoveObjectMessage::~MoveObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRAVEInterface::MoveObjectMessage::MoveObjectMessage(const MoveObjectMessage *m) : Message("MoveObjectMessage")
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
OpenRAVEInterface::MoveObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::MoveObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRAVEInterface::MoveObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name));
}

/** Get x value.
 * x position of object (meters)
 * @return x value
 */
float
OpenRAVEInterface::MoveObjectMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::MoveObjectMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * x position of object (meters)
 * @param new_x new x value
 */
void
OpenRAVEInterface::MoveObjectMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * y position of object (meters)
 * @return y value
 */
float
OpenRAVEInterface::MoveObjectMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::MoveObjectMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * y position of object (meters)
 * @param new_y new y value
 */
void
OpenRAVEInterface::MoveObjectMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get z value.
 * z position of object (meters)
 * @return z value
 */
float
OpenRAVEInterface::MoveObjectMessage::z() const
{
  return data->z;
}

/** Get maximum length of z value.
 * @return length of z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::MoveObjectMessage::maxlenof_z() const
{
  return 1;
}

/** Set z value.
 * z position of object (meters)
 * @param new_z new z value
 */
void
OpenRAVEInterface::MoveObjectMessage::set_z(const float new_z)
{
  data->z = new_z;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRAVEInterface::MoveObjectMessage::clone() const
{
  return new OpenRAVEInterface::MoveObjectMessage(this);
}
/** @class OpenRAVEInterface::RotateObjectMessage <interfaces/OpenRAVEInterface.h>
 * RotateObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_z initial value for z
 */
OpenRAVEInterface::RotateObjectMessage::RotateObjectMessage(const char * ini_name, const float ini_x, const float ini_y, const float ini_z) : Message("RotateObjectMessage")
{
  data_size = sizeof(RotateObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RotateObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30);
  data->x = ini_x;
  data->y = ini_y;
  data->z = ini_z;
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
}
/** Constructor */
OpenRAVEInterface::RotateObjectMessage::RotateObjectMessage() : Message("RotateObjectMessage")
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
OpenRAVEInterface::RotateObjectMessage::~RotateObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRAVEInterface::RotateObjectMessage::RotateObjectMessage(const RotateObjectMessage *m) : Message("RotateObjectMessage")
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
OpenRAVEInterface::RotateObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::RotateObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRAVEInterface::RotateObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name));
}

/** Get x value.
 * x-axis rotation of object (rad)
 * @return x value
 */
float
OpenRAVEInterface::RotateObjectMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::RotateObjectMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * x-axis rotation of object (rad)
 * @param new_x new x value
 */
void
OpenRAVEInterface::RotateObjectMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * y-axis rotation of object (rad)
 * @return y value
 */
float
OpenRAVEInterface::RotateObjectMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::RotateObjectMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * y-axis rotation of object (rad)
 * @param new_y new y value
 */
void
OpenRAVEInterface::RotateObjectMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get z value.
 * z-axis rotation of object (rad)
 * @return z value
 */
float
OpenRAVEInterface::RotateObjectMessage::z() const
{
  return data->z;
}

/** Get maximum length of z value.
 * @return length of z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::RotateObjectMessage::maxlenof_z() const
{
  return 1;
}

/** Set z value.
 * z-axis rotation of object (rad)
 * @param new_z new z value
 */
void
OpenRAVEInterface::RotateObjectMessage::set_z(const float new_z)
{
  data->z = new_z;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRAVEInterface::RotateObjectMessage::clone() const
{
  return new OpenRAVEInterface::RotateObjectMessage(this);
}
/** @class OpenRAVEInterface::RenameObjectMessage <interfaces/OpenRAVEInterface.h>
 * RenameObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 * @param ini_newName initial value for newName
 */
OpenRAVEInterface::RenameObjectMessage::RenameObjectMessage(const char * ini_name, const char * ini_newName) : Message("RenameObjectMessage")
{
  data_size = sizeof(RenameObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RenameObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 30);
  strncpy(data->newName, ini_newName, 30);
  add_fieldinfo(IFT_STRING, "name", 30, data->name);
  add_fieldinfo(IFT_STRING, "newName", 30, data->newName);
}
/** Constructor */
OpenRAVEInterface::RenameObjectMessage::RenameObjectMessage() : Message("RenameObjectMessage")
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
OpenRAVEInterface::RenameObjectMessage::~RenameObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenRAVEInterface::RenameObjectMessage::RenameObjectMessage(const RenameObjectMessage *m) : Message("RenameObjectMessage")
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
OpenRAVEInterface::RenameObjectMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::RenameObjectMessage::maxlenof_name() const
{
  return 30;
}

/** Set name value.
 * Name of object
 * @param new_name new name value
 */
void
OpenRAVEInterface::RenameObjectMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name));
}

/** Get newName value.
 * New name of object
 * @return newName value
 */
char *
OpenRAVEInterface::RenameObjectMessage::newName() const
{
  return data->newName;
}

/** Get maximum length of newName value.
 * @return length of newName value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenRAVEInterface::RenameObjectMessage::maxlenof_newName() const
{
  return 30;
}

/** Set newName value.
 * New name of object
 * @param new_newName new newName value
 */
void
OpenRAVEInterface::RenameObjectMessage::set_newName(const char * new_newName)
{
  strncpy(data->newName, new_newName, sizeof(data->newName));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenRAVEInterface::RenameObjectMessage::clone() const
{
  return new OpenRAVEInterface::RenameObjectMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
OpenRAVEInterface::message_valid(const Message *message) const
{
  const AddObjectMessage *m0 = dynamic_cast<const AddObjectMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const DeleteObjectMessage *m1 = dynamic_cast<const DeleteObjectMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const AttachObjectMessage *m2 = dynamic_cast<const AttachObjectMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const ReleaseObjectMessage *m3 = dynamic_cast<const ReleaseObjectMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const ReleaseAllObjectsMessage *m4 = dynamic_cast<const ReleaseAllObjectsMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const MoveObjectMessage *m5 = dynamic_cast<const MoveObjectMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const RotateObjectMessage *m6 = dynamic_cast<const RotateObjectMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const RenameObjectMessage *m7 = dynamic_cast<const RenameObjectMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(OpenRAVEInterface)
/// @endcond


} // end namespace fawkes
