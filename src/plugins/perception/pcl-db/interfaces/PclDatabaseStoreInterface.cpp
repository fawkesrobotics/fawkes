
/***************************************************************************
 *  PclDatabaseStoreInterface.cpp - Fawkes BlackBoard Interface - PclDatabaseStoreInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2012-2014  Tim Niemueller
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

#include <interfaces/PclDatabaseStoreInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class PclDatabaseStoreInterface <interfaces/PclDatabaseStoreInterface.h>
 * PclDatabaseStoreInterface Fawkes BlackBoard Interface.
 * 
      Instruct the pcl-db-store plugin.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
PclDatabaseStoreInterface::PclDatabaseStoreInterface() : Interface()
{
  data_size = sizeof(PclDatabaseStoreInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (PclDatabaseStoreInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_STRING, "error", 256, data->error);
  add_messageinfo("StoreMessage");
  unsigned char tmp_hash[] = {0x80, 0xe0, 0x2f, 0x81, 0x4a, 0x5, 0x5c, 0xec, 0xd8, 0x64, 0xed, 0x5c, 0xee, 0x17, 0x19, 0xa6};
  set_hash(tmp_hash);
}

/** Destructor */
PclDatabaseStoreInterface::~PclDatabaseStoreInterface()
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
PclDatabaseStoreInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseStoreInterface::maxlenof_msgid() const
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
PclDatabaseStoreInterface::set_msgid(const uint32_t new_msgid)
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
PclDatabaseStoreInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseStoreInterface::maxlenof_final() const
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
PclDatabaseStoreInterface::set_final(const bool new_final)
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
PclDatabaseStoreInterface::error() const
{
  return data->error;
}

/** Get maximum length of error value.
 * @return length of error value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseStoreInterface::maxlenof_error() const
{
  return 256;
}

/** Set error value.
 * 
      Error description if reconstruction fails.
    
 * @param new_error new error value
 */
void
PclDatabaseStoreInterface::set_error(const char * new_error)
{
  strncpy(data->error, new_error, sizeof(data->error)-1);
  data->error[sizeof(data->error)-1] = 0;
  data_changed = true;
}

/* =========== message create =========== */
Message *
PclDatabaseStoreInterface::create_message(const char *type) const
{
  if ( strncmp("StoreMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StoreMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
PclDatabaseStoreInterface::copy_values(const Interface *other)
{
  const PclDatabaseStoreInterface *oi = dynamic_cast<const PclDatabaseStoreInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(PclDatabaseStoreInterface_data_t));
}

const char *
PclDatabaseStoreInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class PclDatabaseStoreInterface::StoreMessage <interfaces/PclDatabaseStoreInterface.h>
 * StoreMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_pcl_id initial value for pcl_id
 * @param ini_database initial value for database
 * @param ini_collection initial value for collection
 */
PclDatabaseStoreInterface::StoreMessage::StoreMessage(const char * ini_pcl_id, const char * ini_database, const char * ini_collection) : Message("StoreMessage")
{
  data_size = sizeof(StoreMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StoreMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->pcl_id, ini_pcl_id, 64-1);
  data->pcl_id[64-1] = 0;
  strncpy(data->database, ini_database, 64-1);
  data->database[64-1] = 0;
  strncpy(data->collection, ini_collection, 128-1);
  data->collection[128-1] = 0;
  add_fieldinfo(IFT_STRING, "pcl_id", 64, data->pcl_id);
  add_fieldinfo(IFT_STRING, "database", 64, data->database);
  add_fieldinfo(IFT_STRING, "collection", 128, data->collection);
}
/** Constructor */
PclDatabaseStoreInterface::StoreMessage::StoreMessage() : Message("StoreMessage")
{
  data_size = sizeof(StoreMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StoreMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "pcl_id", 64, data->pcl_id);
  add_fieldinfo(IFT_STRING, "database", 64, data->database);
  add_fieldinfo(IFT_STRING, "collection", 128, data->collection);
}

/** Destructor */
PclDatabaseStoreInterface::StoreMessage::~StoreMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PclDatabaseStoreInterface::StoreMessage::StoreMessage(const StoreMessage *m) : Message("StoreMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StoreMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get pcl_id value.
 * 
      ID of the point cloud to store.
    
 * @return pcl_id value
 */
char *
PclDatabaseStoreInterface::StoreMessage::pcl_id() const
{
  return data->pcl_id;
}

/** Get maximum length of pcl_id value.
 * @return length of pcl_id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseStoreInterface::StoreMessage::maxlenof_pcl_id() const
{
  return 64;
}

/** Set pcl_id value.
 * 
      ID of the point cloud to store.
    
 * @param new_pcl_id new pcl_id value
 */
void
PclDatabaseStoreInterface::StoreMessage::set_pcl_id(const char * new_pcl_id)
{
  strncpy(data->pcl_id, new_pcl_id, sizeof(data->pcl_id)-1);
  data->pcl_id[sizeof(data->pcl_id)-1] = 0;
}

/** Get database value.
 * 
      Database name from which to read the point clouds. If empty will
      use plugin-configured default.
    
 * @return database value
 */
char *
PclDatabaseStoreInterface::StoreMessage::database() const
{
  return data->database;
}

/** Get maximum length of database value.
 * @return length of database value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseStoreInterface::StoreMessage::maxlenof_database() const
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
PclDatabaseStoreInterface::StoreMessage::set_database(const char * new_database)
{
  strncpy(data->database, new_database, sizeof(data->database)-1);
  data->database[sizeof(data->database)-1] = 0;
}

/** Get collection value.
 * 
      Collection name from which to read the point clouds. May NOT
      include the database name.
    
 * @return collection value
 */
char *
PclDatabaseStoreInterface::StoreMessage::collection() const
{
  return data->collection;
}

/** Get maximum length of collection value.
 * @return length of collection value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseStoreInterface::StoreMessage::maxlenof_collection() const
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
PclDatabaseStoreInterface::StoreMessage::set_collection(const char * new_collection)
{
  strncpy(data->collection, new_collection, sizeof(data->collection)-1);
  data->collection[sizeof(data->collection)-1] = 0;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PclDatabaseStoreInterface::StoreMessage::clone() const
{
  return new PclDatabaseStoreInterface::StoreMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
PclDatabaseStoreInterface::message_valid(const Message *message) const
{
  const StoreMessage *m0 = dynamic_cast<const StoreMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(PclDatabaseStoreInterface)
/// @endcond


} // end namespace fawkes
