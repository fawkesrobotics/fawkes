
/***************************************************************************
 *  PclDatabaseRetrieveInterface.cpp - Fawkes BlackBoard Interface - PclDatabaseRetrieveInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2012-2013  Tim Niemueller
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

#include <interfaces/PclDatabaseRetrieveInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class PclDatabaseRetrieveInterface <interfaces/PclDatabaseRetrieveInterface.h>
 * PclDatabaseRetrieveInterface Fawkes BlackBoard Interface.
 * 
      Instruct the pcl-db-retrieve plugin and receive information.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
PclDatabaseRetrieveInterface::PclDatabaseRetrieveInterface() : Interface()
{
  data_size = sizeof(PclDatabaseRetrieveInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (PclDatabaseRetrieveInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_STRING, "error", 256, data->error);
  add_messageinfo("RetrieveMessage");
  unsigned char tmp_hash[] = {0x9d, 0xf8, 0x9d, 0x53, 0xd7, 0x74, 0xc1, 0x96, 0x38, 0x39, 0xd3, 0xee, 0x76, 0x54, 0x88, 0x75};
  set_hash(tmp_hash);
}

/** Destructor */
PclDatabaseRetrieveInterface::~PclDatabaseRetrieveInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get msgid value.
 * 
      The ID of the message that is currently being processed, or 0 if
      no message is being processed.
    
 * @return msgid value
 */
uint32_t
PclDatabaseRetrieveInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseRetrieveInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * 
      The ID of the message that is currently being processed, or 0 if
      no message is being processed.
    
 * @param new_msgid new msgid value
 */
void
PclDatabaseRetrieveInterface::set_msgid(const uint32_t new_msgid)
{
  data->msgid = new_msgid;
  data_changed = true;
}

/** Get final value.
 * 
      True, if the last goto command has been finished, false if it is
      still running.
    
 * @return final value
 */
bool
PclDatabaseRetrieveInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseRetrieveInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * 
      True, if the last goto command has been finished, false if it is
      still running.
    
 * @param new_final new final value
 */
void
PclDatabaseRetrieveInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/** Get error value.
 * 
      Error description if reconstruction fails.
    
 * @return error value
 */
char *
PclDatabaseRetrieveInterface::error() const
{
  return data->error;
}

/** Get maximum length of error value.
 * @return length of error value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseRetrieveInterface::maxlenof_error() const
{
  return 256;
}

/** Set error value.
 * 
      Error description if reconstruction fails.
    
 * @param new_error new error value
 */
void
PclDatabaseRetrieveInterface::set_error(const char * new_error)
{
  strncpy(data->error, new_error, sizeof(data->error));
  data_changed = true;
}

/* =========== message create =========== */
Message *
PclDatabaseRetrieveInterface::create_message(const char *type) const
{
  if ( strncmp("RetrieveMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RetrieveMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
PclDatabaseRetrieveInterface::copy_values(const Interface *other)
{
  const PclDatabaseRetrieveInterface *oi = dynamic_cast<const PclDatabaseRetrieveInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(PclDatabaseRetrieveInterface_data_t));
}

const char *
PclDatabaseRetrieveInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class PclDatabaseRetrieveInterface::RetrieveMessage <interfaces/PclDatabaseRetrieveInterface.h>
 * RetrieveMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_timestamp initial value for timestamp
 * @param ini_database initial value for database
 * @param ini_collection initial value for collection
 * @param ini_target_frame initial value for target_frame
 * @param ini_original_timestamp initial value for original_timestamp
 */
PclDatabaseRetrieveInterface::RetrieveMessage::RetrieveMessage(const int64_t ini_timestamp, const char * ini_database, const char * ini_collection, const char * ini_target_frame, const bool ini_original_timestamp) : Message("RetrieveMessage")
{
  data_size = sizeof(RetrieveMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RetrieveMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->timestamp = ini_timestamp;
  strncpy(data->database, ini_database, 64);
  strncpy(data->collection, ini_collection, 128);
  strncpy(data->target_frame, ini_target_frame, 64);
  data->original_timestamp = ini_original_timestamp;
  add_fieldinfo(IFT_INT64, "timestamp", 1, &data->timestamp);
  add_fieldinfo(IFT_STRING, "database", 64, data->database);
  add_fieldinfo(IFT_STRING, "collection", 128, data->collection);
  add_fieldinfo(IFT_STRING, "target_frame", 64, data->target_frame);
  add_fieldinfo(IFT_BOOL, "original_timestamp", 1, &data->original_timestamp);
}
/** Constructor */
PclDatabaseRetrieveInterface::RetrieveMessage::RetrieveMessage() : Message("RetrieveMessage")
{
  data_size = sizeof(RetrieveMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RetrieveMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_INT64, "timestamp", 1, &data->timestamp);
  add_fieldinfo(IFT_STRING, "database", 64, data->database);
  add_fieldinfo(IFT_STRING, "collection", 128, data->collection);
  add_fieldinfo(IFT_STRING, "target_frame", 64, data->target_frame);
  add_fieldinfo(IFT_BOOL, "original_timestamp", 1, &data->original_timestamp);
}

/** Destructor */
PclDatabaseRetrieveInterface::RetrieveMessage::~RetrieveMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PclDatabaseRetrieveInterface::RetrieveMessage::RetrieveMessage(const RetrieveMessage *m) : Message("RetrieveMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (RetrieveMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get timestamp value.
 * 
      Timestamp for which to retrieve the most recent point clouds.
    
 * @return timestamp value
 */
int64_t
PclDatabaseRetrieveInterface::RetrieveMessage::timestamp() const
{
  return data->timestamp;
}

/** Get maximum length of timestamp value.
 * @return length of timestamp value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseRetrieveInterface::RetrieveMessage::maxlenof_timestamp() const
{
  return 1;
}

/** Set timestamp value.
 * 
      Timestamp for which to retrieve the most recent point clouds.
    
 * @param new_timestamp new timestamp value
 */
void
PclDatabaseRetrieveInterface::RetrieveMessage::set_timestamp(const int64_t new_timestamp)
{
  data->timestamp = new_timestamp;
}

/** Get database value.
 * 
      Database name from which to read the point clouds. If empty will
      use plugin-configured default.
    
 * @return database value
 */
char *
PclDatabaseRetrieveInterface::RetrieveMessage::database() const
{
  return data->database;
}

/** Get maximum length of database value.
 * @return length of database value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseRetrieveInterface::RetrieveMessage::maxlenof_database() const
{
  return 64;
}

/** Set database value.
 * 
      Database name from which to read the point clouds. If empty will
      use plugin-configured default.
    
 * @param new_database new database value
 */
void
PclDatabaseRetrieveInterface::RetrieveMessage::set_database(const char * new_database)
{
  strncpy(data->database, new_database, sizeof(data->database));
}

/** Get collection value.
 * 
      Collection name from which to read the point clouds. May NOT
      include the database name.
    
 * @return collection value
 */
char *
PclDatabaseRetrieveInterface::RetrieveMessage::collection() const
{
  return data->collection;
}

/** Get maximum length of collection value.
 * @return length of collection value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseRetrieveInterface::RetrieveMessage::maxlenof_collection() const
{
  return 128;
}

/** Set collection value.
 * 
      Collection name from which to read the point clouds. May NOT
      include the database name.
    
 * @param new_collection new collection value
 */
void
PclDatabaseRetrieveInterface::RetrieveMessage::set_collection(const char * new_collection)
{
  strncpy(data->collection, new_collection, sizeof(data->collection));
}

/** Get target_frame value.
 * 
      Coordinate frame to which to transform the output point cloud.
      The transformation will be done through a fixed frame specified
      in the plugin config. If empty, no transformation is
      performed. If set to "SENSOR" will convert to the sensor frame
      specified in the plugin config.
    
 * @return target_frame value
 */
char *
PclDatabaseRetrieveInterface::RetrieveMessage::target_frame() const
{
  return data->target_frame;
}

/** Get maximum length of target_frame value.
 * @return length of target_frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseRetrieveInterface::RetrieveMessage::maxlenof_target_frame() const
{
  return 64;
}

/** Set target_frame value.
 * 
      Coordinate frame to which to transform the output point cloud.
      The transformation will be done through a fixed frame specified
      in the plugin config. If empty, no transformation is
      performed. If set to "SENSOR" will convert to the sensor frame
      specified in the plugin config.
    
 * @param new_target_frame new target_frame value
 */
void
PclDatabaseRetrieveInterface::RetrieveMessage::set_target_frame(const char * new_target_frame)
{
  strncpy(data->target_frame, new_target_frame, sizeof(data->target_frame));
}

/** Get original_timestamp value.
 * 
      Set to true to set the original timestamp on the point cloud,
      false (default) to publish with current time.
    
 * @return original_timestamp value
 */
bool
PclDatabaseRetrieveInterface::RetrieveMessage::is_original_timestamp() const
{
  return data->original_timestamp;
}

/** Get maximum length of original_timestamp value.
 * @return length of original_timestamp value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseRetrieveInterface::RetrieveMessage::maxlenof_original_timestamp() const
{
  return 1;
}

/** Set original_timestamp value.
 * 
      Set to true to set the original timestamp on the point cloud,
      false (default) to publish with current time.
    
 * @param new_original_timestamp new original_timestamp value
 */
void
PclDatabaseRetrieveInterface::RetrieveMessage::set_original_timestamp(const bool new_original_timestamp)
{
  data->original_timestamp = new_original_timestamp;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PclDatabaseRetrieveInterface::RetrieveMessage::clone() const
{
  return new PclDatabaseRetrieveInterface::RetrieveMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
PclDatabaseRetrieveInterface::message_valid(const Message *message) const
{
  const RetrieveMessage *m0 = dynamic_cast<const RetrieveMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(PclDatabaseRetrieveInterface)
/// @endcond


} // end namespace fawkes
