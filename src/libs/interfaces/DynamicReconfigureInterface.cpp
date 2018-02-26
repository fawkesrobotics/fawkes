
/***************************************************************************
 *  DynamicReconfigureInterface.cpp - Fawkes BlackBoard Interface - DynamicReconfigureInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2017  Christoph Henke
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

#include <interfaces/DynamicReconfigureInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class DynamicReconfigureInterface <interfaces/DynamicReconfigureInterface.h>
 * DynamicReconfigureInterface Fawkes BlackBoard Interface.
 * 
      Currently only the last set parameter is displayed.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
DynamicReconfigureInterface::DynamicReconfigureInterface() : Interface()
{
  data_size = sizeof(DynamicReconfigureInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (DynamicReconfigureInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_LastMsgStatus[(int)Succeeded] = "Succeeded";
  enum_map_LastMsgStatus[(int)Failed] = "Failed";
  add_fieldinfo(IFT_STRING, "last_service", 64, data->last_service);
  add_fieldinfo(IFT_STRING, "last_parameter", 64, data->last_parameter);
  add_fieldinfo(IFT_BOOL, "last_bool_value", 1, &data->last_bool_value);
  add_fieldinfo(IFT_STRING, "last_str_value", 64, data->last_str_value);
  add_fieldinfo(IFT_UINT32, "last_uint32_value", 1, &data->last_uint32_value);
  add_fieldinfo(IFT_UINT64, "last_uint64_value", 1, &data->last_uint64_value);
  add_fieldinfo(IFT_FLOAT, "last_float_value", 1, &data->last_float_value);
  add_fieldinfo(IFT_UINT64, "last_msg_id", 1, &data->last_msg_id);
  add_fieldinfo(IFT_ENUM, "last_msg_status", 1, &data->last_msg_status, "LastMsgStatus", &enum_map_LastMsgStatus);
  add_messageinfo("SetBoolMessage");
  add_messageinfo("SetStringMessage");
  add_messageinfo("SetUint32Message");
  add_messageinfo("SetUint64Message");
  add_messageinfo("SetFloatMessage");
  unsigned char tmp_hash[] = {0xe2, 0xc2, 0x78, 0x4a, 0x9f, 0x90, 0x36, 0x57, 0xeb, 0x26, 0x9, 0xe0, 0xe8, 0x77, 0xcd, 0x9d};
  set_hash(tmp_hash);
}

/** Destructor */
DynamicReconfigureInterface::~DynamicReconfigureInterface()
{
  free(data_ptr);
}
/** Convert LastMsgStatus constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
DynamicReconfigureInterface::tostring_LastMsgStatus(LastMsgStatus value) const
{
  switch (value) {
  case Succeeded: return "Succeeded";
  case Failed: return "Failed";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get last_service value.
 * The last service for reconfiguration called.
 * @return last_service value
 */
char *
DynamicReconfigureInterface::last_service() const
{
  return data->last_service;
}

/** Get maximum length of last_service value.
 * @return length of last_service value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::maxlenof_last_service() const
{
  return 64;
}

/** Set last_service value.
 * The last service for reconfiguration called.
 * @param new_last_service new last_service value
 */
void
DynamicReconfigureInterface::set_last_service(const char * new_last_service)
{
  strncpy(data->last_service, new_last_service, sizeof(data->last_service)-1);
  data->last_service[sizeof(data->last_service)-1] = 0;
  data_changed = true;
}

/** Get last_parameter value.
 * The last parameter name.
 * @return last_parameter value
 */
char *
DynamicReconfigureInterface::last_parameter() const
{
  return data->last_parameter;
}

/** Get maximum length of last_parameter value.
 * @return length of last_parameter value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::maxlenof_last_parameter() const
{
  return 64;
}

/** Set last_parameter value.
 * The last parameter name.
 * @param new_last_parameter new last_parameter value
 */
void
DynamicReconfigureInterface::set_last_parameter(const char * new_last_parameter)
{
  strncpy(data->last_parameter, new_last_parameter, sizeof(data->last_parameter)-1);
  data->last_parameter[sizeof(data->last_parameter)-1] = 0;
  data_changed = true;
}

/** Get last_bool_value value.
 * The last parameter value.
 * @return last_bool_value value
 */
bool
DynamicReconfigureInterface::is_last_bool_value() const
{
  return data->last_bool_value;
}

/** Get maximum length of last_bool_value value.
 * @return length of last_bool_value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::maxlenof_last_bool_value() const
{
  return 1;
}

/** Set last_bool_value value.
 * The last parameter value.
 * @param new_last_bool_value new last_bool_value value
 */
void
DynamicReconfigureInterface::set_last_bool_value(const bool new_last_bool_value)
{
  data->last_bool_value = new_last_bool_value;
  data_changed = true;
}

/** Get last_str_value value.
 * The last parameter value.
 * @return last_str_value value
 */
char *
DynamicReconfigureInterface::last_str_value() const
{
  return data->last_str_value;
}

/** Get maximum length of last_str_value value.
 * @return length of last_str_value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::maxlenof_last_str_value() const
{
  return 64;
}

/** Set last_str_value value.
 * The last parameter value.
 * @param new_last_str_value new last_str_value value
 */
void
DynamicReconfigureInterface::set_last_str_value(const char * new_last_str_value)
{
  strncpy(data->last_str_value, new_last_str_value, sizeof(data->last_str_value)-1);
  data->last_str_value[sizeof(data->last_str_value)-1] = 0;
  data_changed = true;
}

/** Get last_uint32_value value.
 * The last parameter value.
 * @return last_uint32_value value
 */
uint32_t
DynamicReconfigureInterface::last_uint32_value() const
{
  return data->last_uint32_value;
}

/** Get maximum length of last_uint32_value value.
 * @return length of last_uint32_value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::maxlenof_last_uint32_value() const
{
  return 1;
}

/** Set last_uint32_value value.
 * The last parameter value.
 * @param new_last_uint32_value new last_uint32_value value
 */
void
DynamicReconfigureInterface::set_last_uint32_value(const uint32_t new_last_uint32_value)
{
  data->last_uint32_value = new_last_uint32_value;
  data_changed = true;
}

/** Get last_uint64_value value.
 * The last parameter value.
 * @return last_uint64_value value
 */
uint64_t
DynamicReconfigureInterface::last_uint64_value() const
{
  return data->last_uint64_value;
}

/** Get maximum length of last_uint64_value value.
 * @return length of last_uint64_value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::maxlenof_last_uint64_value() const
{
  return 1;
}

/** Set last_uint64_value value.
 * The last parameter value.
 * @param new_last_uint64_value new last_uint64_value value
 */
void
DynamicReconfigureInterface::set_last_uint64_value(const uint64_t new_last_uint64_value)
{
  data->last_uint64_value = new_last_uint64_value;
  data_changed = true;
}

/** Get last_float_value value.
 * The last parameter value.
 * @return last_float_value value
 */
float
DynamicReconfigureInterface::last_float_value() const
{
  return data->last_float_value;
}

/** Get maximum length of last_float_value value.
 * @return length of last_float_value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::maxlenof_last_float_value() const
{
  return 1;
}

/** Set last_float_value value.
 * The last parameter value.
 * @param new_last_float_value new last_float_value value
 */
void
DynamicReconfigureInterface::set_last_float_value(const float new_last_float_value)
{
  data->last_float_value = new_last_float_value;
  data_changed = true;
}

/** Get last_msg_id value.
 * The last parameter name.
 * @return last_msg_id value
 */
uint64_t
DynamicReconfigureInterface::last_msg_id() const
{
  return data->last_msg_id;
}

/** Get maximum length of last_msg_id value.
 * @return length of last_msg_id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::maxlenof_last_msg_id() const
{
  return 1;
}

/** Set last_msg_id value.
 * The last parameter name.
 * @param new_last_msg_id new last_msg_id value
 */
void
DynamicReconfigureInterface::set_last_msg_id(const uint64_t new_last_msg_id)
{
  data->last_msg_id = new_last_msg_id;
  data_changed = true;
}

/** Get last_msg_status value.
 * The last send message status.
 * @return last_msg_status value
 */
DynamicReconfigureInterface::LastMsgStatus
DynamicReconfigureInterface::last_msg_status() const
{
  return (DynamicReconfigureInterface::LastMsgStatus)data->last_msg_status;
}

/** Get maximum length of last_msg_status value.
 * @return length of last_msg_status value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::maxlenof_last_msg_status() const
{
  return 1;
}

/** Set last_msg_status value.
 * The last send message status.
 * @param new_last_msg_status new last_msg_status value
 */
void
DynamicReconfigureInterface::set_last_msg_status(const LastMsgStatus new_last_msg_status)
{
  data->last_msg_status = new_last_msg_status;
  data_changed = true;
}

/* =========== message create =========== */
Message *
DynamicReconfigureInterface::create_message(const char *type) const
{
  if ( strncmp("SetBoolMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetBoolMessage();
  } else if ( strncmp("SetStringMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetStringMessage();
  } else if ( strncmp("SetUint32Message", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetUint32Message();
  } else if ( strncmp("SetUint64Message", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetUint64Message();
  } else if ( strncmp("SetFloatMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetFloatMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
DynamicReconfigureInterface::copy_values(const Interface *other)
{
  const DynamicReconfigureInterface *oi = dynamic_cast<const DynamicReconfigureInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(DynamicReconfigureInterface_data_t));
}

const char *
DynamicReconfigureInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "LastMsgStatus") == 0) {
    return tostring_LastMsgStatus((LastMsgStatus)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class DynamicReconfigureInterface::SetBoolMessage <interfaces/DynamicReconfigureInterface.h>
 * SetBoolMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_service initial value for service
 * @param ini_parameter initial value for parameter
 * @param ini_value initial value for value
 */
DynamicReconfigureInterface::SetBoolMessage::SetBoolMessage(const char * ini_service, const char * ini_parameter, const bool ini_value) : Message("SetBoolMessage")
{
  data_size = sizeof(SetBoolMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetBoolMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->service, ini_service, 64-1);
  data->service[64-1] = 0;
  strncpy(data->parameter, ini_parameter, 64-1);
  data->parameter[64-1] = 0;
  data->value = ini_value;
  enum_map_LastMsgStatus[(int)Succeeded] = "Succeeded";
  enum_map_LastMsgStatus[(int)Failed] = "Failed";
  add_fieldinfo(IFT_STRING, "service", 64, data->service);
  add_fieldinfo(IFT_STRING, "parameter", 64, data->parameter);
  add_fieldinfo(IFT_BOOL, "value", 1, &data->value);
}
/** Constructor */
DynamicReconfigureInterface::SetBoolMessage::SetBoolMessage() : Message("SetBoolMessage")
{
  data_size = sizeof(SetBoolMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetBoolMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_LastMsgStatus[(int)Succeeded] = "Succeeded";
  enum_map_LastMsgStatus[(int)Failed] = "Failed";
  add_fieldinfo(IFT_STRING, "service", 64, data->service);
  add_fieldinfo(IFT_STRING, "parameter", 64, data->parameter);
  add_fieldinfo(IFT_BOOL, "value", 1, &data->value);
}

/** Destructor */
DynamicReconfigureInterface::SetBoolMessage::~SetBoolMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamicReconfigureInterface::SetBoolMessage::SetBoolMessage(const SetBoolMessage *m) : Message("SetBoolMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetBoolMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get service value.
 * Name of the ROS service for dynamic reconfiguration.
 * @return service value
 */
char *
DynamicReconfigureInterface::SetBoolMessage::service() const
{
  return data->service;
}

/** Get maximum length of service value.
 * @return length of service value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetBoolMessage::maxlenof_service() const
{
  return 64;
}

/** Set service value.
 * Name of the ROS service for dynamic reconfiguration.
 * @param new_service new service value
 */
void
DynamicReconfigureInterface::SetBoolMessage::set_service(const char * new_service)
{
  strncpy(data->service, new_service, sizeof(data->service)-1);
  data->service[sizeof(data->service)-1] = 0;
}

/** Get parameter value.
 * Name of the ROS parameter.
 * @return parameter value
 */
char *
DynamicReconfigureInterface::SetBoolMessage::parameter() const
{
  return data->parameter;
}

/** Get maximum length of parameter value.
 * @return length of parameter value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetBoolMessage::maxlenof_parameter() const
{
  return 64;
}

/** Set parameter value.
 * Name of the ROS parameter.
 * @param new_parameter new parameter value
 */
void
DynamicReconfigureInterface::SetBoolMessage::set_parameter(const char * new_parameter)
{
  strncpy(data->parameter, new_parameter, sizeof(data->parameter)-1);
  data->parameter[sizeof(data->parameter)-1] = 0;
}

/** Get value value.
 * The bool value.
 * @return value value
 */
bool
DynamicReconfigureInterface::SetBoolMessage::is_value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetBoolMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * The bool value.
 * @param new_value new value value
 */
void
DynamicReconfigureInterface::SetBoolMessage::set_value(const bool new_value)
{
  data->value = new_value;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamicReconfigureInterface::SetBoolMessage::clone() const
{
  return new DynamicReconfigureInterface::SetBoolMessage(this);
}
/** @class DynamicReconfigureInterface::SetStringMessage <interfaces/DynamicReconfigureInterface.h>
 * SetStringMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_service initial value for service
 * @param ini_parameter initial value for parameter
 * @param ini_value initial value for value
 */
DynamicReconfigureInterface::SetStringMessage::SetStringMessage(const char * ini_service, const char * ini_parameter, const char * ini_value) : Message("SetStringMessage")
{
  data_size = sizeof(SetStringMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetStringMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->service, ini_service, 64-1);
  data->service[64-1] = 0;
  strncpy(data->parameter, ini_parameter, 64-1);
  data->parameter[64-1] = 0;
  strncpy(data->value, ini_value, 64-1);
  data->value[64-1] = 0;
  enum_map_LastMsgStatus[(int)Succeeded] = "Succeeded";
  enum_map_LastMsgStatus[(int)Failed] = "Failed";
  add_fieldinfo(IFT_STRING, "service", 64, data->service);
  add_fieldinfo(IFT_STRING, "parameter", 64, data->parameter);
  add_fieldinfo(IFT_STRING, "value", 64, data->value);
}
/** Constructor */
DynamicReconfigureInterface::SetStringMessage::SetStringMessage() : Message("SetStringMessage")
{
  data_size = sizeof(SetStringMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetStringMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_LastMsgStatus[(int)Succeeded] = "Succeeded";
  enum_map_LastMsgStatus[(int)Failed] = "Failed";
  add_fieldinfo(IFT_STRING, "service", 64, data->service);
  add_fieldinfo(IFT_STRING, "parameter", 64, data->parameter);
  add_fieldinfo(IFT_STRING, "value", 64, data->value);
}

/** Destructor */
DynamicReconfigureInterface::SetStringMessage::~SetStringMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamicReconfigureInterface::SetStringMessage::SetStringMessage(const SetStringMessage *m) : Message("SetStringMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetStringMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get service value.
 * Name of the ROS service for dynamic reconfiguration.
 * @return service value
 */
char *
DynamicReconfigureInterface::SetStringMessage::service() const
{
  return data->service;
}

/** Get maximum length of service value.
 * @return length of service value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetStringMessage::maxlenof_service() const
{
  return 64;
}

/** Set service value.
 * Name of the ROS service for dynamic reconfiguration.
 * @param new_service new service value
 */
void
DynamicReconfigureInterface::SetStringMessage::set_service(const char * new_service)
{
  strncpy(data->service, new_service, sizeof(data->service)-1);
  data->service[sizeof(data->service)-1] = 0;
}

/** Get parameter value.
 * Name of the ROS parameter.
 * @return parameter value
 */
char *
DynamicReconfigureInterface::SetStringMessage::parameter() const
{
  return data->parameter;
}

/** Get maximum length of parameter value.
 * @return length of parameter value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetStringMessage::maxlenof_parameter() const
{
  return 64;
}

/** Set parameter value.
 * Name of the ROS parameter.
 * @param new_parameter new parameter value
 */
void
DynamicReconfigureInterface::SetStringMessage::set_parameter(const char * new_parameter)
{
  strncpy(data->parameter, new_parameter, sizeof(data->parameter)-1);
  data->parameter[sizeof(data->parameter)-1] = 0;
}

/** Get value value.
 * The value to set.
 * @return value value
 */
char *
DynamicReconfigureInterface::SetStringMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetStringMessage::maxlenof_value() const
{
  return 64;
}

/** Set value value.
 * The value to set.
 * @param new_value new value value
 */
void
DynamicReconfigureInterface::SetStringMessage::set_value(const char * new_value)
{
  strncpy(data->value, new_value, sizeof(data->value)-1);
  data->value[sizeof(data->value)-1] = 0;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamicReconfigureInterface::SetStringMessage::clone() const
{
  return new DynamicReconfigureInterface::SetStringMessage(this);
}
/** @class DynamicReconfigureInterface::SetUint32Message <interfaces/DynamicReconfigureInterface.h>
 * SetUint32Message Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_service initial value for service
 * @param ini_parameter initial value for parameter
 * @param ini_value initial value for value
 */
DynamicReconfigureInterface::SetUint32Message::SetUint32Message(const char * ini_service, const char * ini_parameter, const uint32_t ini_value) : Message("SetUint32Message")
{
  data_size = sizeof(SetUint32Message_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetUint32Message_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->service, ini_service, 64-1);
  data->service[64-1] = 0;
  strncpy(data->parameter, ini_parameter, 64-1);
  data->parameter[64-1] = 0;
  data->value = ini_value;
  enum_map_LastMsgStatus[(int)Succeeded] = "Succeeded";
  enum_map_LastMsgStatus[(int)Failed] = "Failed";
  add_fieldinfo(IFT_STRING, "service", 64, data->service);
  add_fieldinfo(IFT_STRING, "parameter", 64, data->parameter);
  add_fieldinfo(IFT_UINT32, "value", 1, &data->value);
}
/** Constructor */
DynamicReconfigureInterface::SetUint32Message::SetUint32Message() : Message("SetUint32Message")
{
  data_size = sizeof(SetUint32Message_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetUint32Message_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_LastMsgStatus[(int)Succeeded] = "Succeeded";
  enum_map_LastMsgStatus[(int)Failed] = "Failed";
  add_fieldinfo(IFT_STRING, "service", 64, data->service);
  add_fieldinfo(IFT_STRING, "parameter", 64, data->parameter);
  add_fieldinfo(IFT_UINT32, "value", 1, &data->value);
}

/** Destructor */
DynamicReconfigureInterface::SetUint32Message::~SetUint32Message()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamicReconfigureInterface::SetUint32Message::SetUint32Message(const SetUint32Message *m) : Message("SetUint32Message")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetUint32Message_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get service value.
 * Name of the ROS service for dynamic reconfiguration.
 * @return service value
 */
char *
DynamicReconfigureInterface::SetUint32Message::service() const
{
  return data->service;
}

/** Get maximum length of service value.
 * @return length of service value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetUint32Message::maxlenof_service() const
{
  return 64;
}

/** Set service value.
 * Name of the ROS service for dynamic reconfiguration.
 * @param new_service new service value
 */
void
DynamicReconfigureInterface::SetUint32Message::set_service(const char * new_service)
{
  strncpy(data->service, new_service, sizeof(data->service)-1);
  data->service[sizeof(data->service)-1] = 0;
}

/** Get parameter value.
 * Name of the ROS parameter.
 * @return parameter value
 */
char *
DynamicReconfigureInterface::SetUint32Message::parameter() const
{
  return data->parameter;
}

/** Get maximum length of parameter value.
 * @return length of parameter value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetUint32Message::maxlenof_parameter() const
{
  return 64;
}

/** Set parameter value.
 * Name of the ROS parameter.
 * @param new_parameter new parameter value
 */
void
DynamicReconfigureInterface::SetUint32Message::set_parameter(const char * new_parameter)
{
  strncpy(data->parameter, new_parameter, sizeof(data->parameter)-1);
  data->parameter[sizeof(data->parameter)-1] = 0;
}

/** Get value value.
 * The value to set.
 * @return value value
 */
uint32_t
DynamicReconfigureInterface::SetUint32Message::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetUint32Message::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * The value to set.
 * @param new_value new value value
 */
void
DynamicReconfigureInterface::SetUint32Message::set_value(const uint32_t new_value)
{
  data->value = new_value;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamicReconfigureInterface::SetUint32Message::clone() const
{
  return new DynamicReconfigureInterface::SetUint32Message(this);
}
/** @class DynamicReconfigureInterface::SetUint64Message <interfaces/DynamicReconfigureInterface.h>
 * SetUint64Message Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_service initial value for service
 * @param ini_parameter initial value for parameter
 * @param ini_value initial value for value
 */
DynamicReconfigureInterface::SetUint64Message::SetUint64Message(const char * ini_service, const char * ini_parameter, const uint64_t ini_value) : Message("SetUint64Message")
{
  data_size = sizeof(SetUint64Message_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetUint64Message_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->service, ini_service, 64-1);
  data->service[64-1] = 0;
  strncpy(data->parameter, ini_parameter, 64-1);
  data->parameter[64-1] = 0;
  data->value = ini_value;
  enum_map_LastMsgStatus[(int)Succeeded] = "Succeeded";
  enum_map_LastMsgStatus[(int)Failed] = "Failed";
  add_fieldinfo(IFT_STRING, "service", 64, data->service);
  add_fieldinfo(IFT_STRING, "parameter", 64, data->parameter);
  add_fieldinfo(IFT_UINT64, "value", 1, &data->value);
}
/** Constructor */
DynamicReconfigureInterface::SetUint64Message::SetUint64Message() : Message("SetUint64Message")
{
  data_size = sizeof(SetUint64Message_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetUint64Message_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_LastMsgStatus[(int)Succeeded] = "Succeeded";
  enum_map_LastMsgStatus[(int)Failed] = "Failed";
  add_fieldinfo(IFT_STRING, "service", 64, data->service);
  add_fieldinfo(IFT_STRING, "parameter", 64, data->parameter);
  add_fieldinfo(IFT_UINT64, "value", 1, &data->value);
}

/** Destructor */
DynamicReconfigureInterface::SetUint64Message::~SetUint64Message()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamicReconfigureInterface::SetUint64Message::SetUint64Message(const SetUint64Message *m) : Message("SetUint64Message")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetUint64Message_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get service value.
 * Name of the ROS service for dynamic reconfiguration.
 * @return service value
 */
char *
DynamicReconfigureInterface::SetUint64Message::service() const
{
  return data->service;
}

/** Get maximum length of service value.
 * @return length of service value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetUint64Message::maxlenof_service() const
{
  return 64;
}

/** Set service value.
 * Name of the ROS service for dynamic reconfiguration.
 * @param new_service new service value
 */
void
DynamicReconfigureInterface::SetUint64Message::set_service(const char * new_service)
{
  strncpy(data->service, new_service, sizeof(data->service)-1);
  data->service[sizeof(data->service)-1] = 0;
}

/** Get parameter value.
 * Name of the ROS parameter.
 * @return parameter value
 */
char *
DynamicReconfigureInterface::SetUint64Message::parameter() const
{
  return data->parameter;
}

/** Get maximum length of parameter value.
 * @return length of parameter value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetUint64Message::maxlenof_parameter() const
{
  return 64;
}

/** Set parameter value.
 * Name of the ROS parameter.
 * @param new_parameter new parameter value
 */
void
DynamicReconfigureInterface::SetUint64Message::set_parameter(const char * new_parameter)
{
  strncpy(data->parameter, new_parameter, sizeof(data->parameter)-1);
  data->parameter[sizeof(data->parameter)-1] = 0;
}

/** Get value value.
 * The value to set.
 * @return value value
 */
uint64_t
DynamicReconfigureInterface::SetUint64Message::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetUint64Message::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * The value to set.
 * @param new_value new value value
 */
void
DynamicReconfigureInterface::SetUint64Message::set_value(const uint64_t new_value)
{
  data->value = new_value;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamicReconfigureInterface::SetUint64Message::clone() const
{
  return new DynamicReconfigureInterface::SetUint64Message(this);
}
/** @class DynamicReconfigureInterface::SetFloatMessage <interfaces/DynamicReconfigureInterface.h>
 * SetFloatMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_service initial value for service
 * @param ini_parameter initial value for parameter
 * @param ini_value initial value for value
 */
DynamicReconfigureInterface::SetFloatMessage::SetFloatMessage(const char * ini_service, const char * ini_parameter, const float ini_value) : Message("SetFloatMessage")
{
  data_size = sizeof(SetFloatMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetFloatMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->service, ini_service, 64-1);
  data->service[64-1] = 0;
  strncpy(data->parameter, ini_parameter, 64-1);
  data->parameter[64-1] = 0;
  data->value = ini_value;
  enum_map_LastMsgStatus[(int)Succeeded] = "Succeeded";
  enum_map_LastMsgStatus[(int)Failed] = "Failed";
  add_fieldinfo(IFT_STRING, "service", 64, data->service);
  add_fieldinfo(IFT_STRING, "parameter", 64, data->parameter);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
}
/** Constructor */
DynamicReconfigureInterface::SetFloatMessage::SetFloatMessage() : Message("SetFloatMessage")
{
  data_size = sizeof(SetFloatMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetFloatMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_LastMsgStatus[(int)Succeeded] = "Succeeded";
  enum_map_LastMsgStatus[(int)Failed] = "Failed";
  add_fieldinfo(IFT_STRING, "service", 64, data->service);
  add_fieldinfo(IFT_STRING, "parameter", 64, data->parameter);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
}

/** Destructor */
DynamicReconfigureInterface::SetFloatMessage::~SetFloatMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamicReconfigureInterface::SetFloatMessage::SetFloatMessage(const SetFloatMessage *m) : Message("SetFloatMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetFloatMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get service value.
 * Name of the ROS service for dynamic reconfiguration.
 * @return service value
 */
char *
DynamicReconfigureInterface::SetFloatMessage::service() const
{
  return data->service;
}

/** Get maximum length of service value.
 * @return length of service value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetFloatMessage::maxlenof_service() const
{
  return 64;
}

/** Set service value.
 * Name of the ROS service for dynamic reconfiguration.
 * @param new_service new service value
 */
void
DynamicReconfigureInterface::SetFloatMessage::set_service(const char * new_service)
{
  strncpy(data->service, new_service, sizeof(data->service)-1);
  data->service[sizeof(data->service)-1] = 0;
}

/** Get parameter value.
 * Name of the ROS parameter.
 * @return parameter value
 */
char *
DynamicReconfigureInterface::SetFloatMessage::parameter() const
{
  return data->parameter;
}

/** Get maximum length of parameter value.
 * @return length of parameter value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetFloatMessage::maxlenof_parameter() const
{
  return 64;
}

/** Set parameter value.
 * Name of the ROS parameter.
 * @param new_parameter new parameter value
 */
void
DynamicReconfigureInterface::SetFloatMessage::set_parameter(const char * new_parameter)
{
  strncpy(data->parameter, new_parameter, sizeof(data->parameter)-1);
  data->parameter[sizeof(data->parameter)-1] = 0;
}

/** Get value value.
 * The value to set.
 * @return value value
 */
float
DynamicReconfigureInterface::SetFloatMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamicReconfigureInterface::SetFloatMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * The value to set.
 * @param new_value new value value
 */
void
DynamicReconfigureInterface::SetFloatMessage::set_value(const float new_value)
{
  data->value = new_value;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamicReconfigureInterface::SetFloatMessage::clone() const
{
  return new DynamicReconfigureInterface::SetFloatMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
DynamicReconfigureInterface::message_valid(const Message *message) const
{
  const SetBoolMessage *m0 = dynamic_cast<const SetBoolMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SetStringMessage *m1 = dynamic_cast<const SetStringMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const SetUint32Message *m2 = dynamic_cast<const SetUint32Message *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const SetUint64Message *m3 = dynamic_cast<const SetUint64Message *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const SetFloatMessage *m4 = dynamic_cast<const SetFloatMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(DynamicReconfigureInterface)
/// @endcond


} // end namespace fawkes
