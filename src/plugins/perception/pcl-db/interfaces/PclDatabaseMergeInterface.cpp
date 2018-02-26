
/***************************************************************************
 *  PclDatabaseMergeInterface.cpp - Fawkes BlackBoard Interface - PclDatabaseMergeInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2012  Tim Niemueller
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

#include <interfaces/PclDatabaseMergeInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class PclDatabaseMergeInterface <interfaces/PclDatabaseMergeInterface.h>
 * PclDatabaseMergeInterface Fawkes BlackBoard Interface.
 * 
      Instruct the pcl-db-merge plugin and receive information.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
PclDatabaseMergeInterface::PclDatabaseMergeInterface() : Interface()
{
  data_size = sizeof(PclDatabaseMergeInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (PclDatabaseMergeInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_STRING, "error", 256, data->error);
  add_messageinfo("MergeMessage");
  unsigned char tmp_hash[] = {0x1a, 0xb, 0xb8, 0x5a, 0x7, 0x88, 0x93, 0x55, 0x9e, 0x7e, 0xcb, 0x96, 0x46, 0x8f, 0x97, 0xb1};
  set_hash(tmp_hash);
}

/** Destructor */
PclDatabaseMergeInterface::~PclDatabaseMergeInterface()
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
PclDatabaseMergeInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseMergeInterface::maxlenof_msgid() const
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
PclDatabaseMergeInterface::set_msgid(const uint32_t new_msgid)
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
PclDatabaseMergeInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseMergeInterface::maxlenof_final() const
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
PclDatabaseMergeInterface::set_final(const bool new_final)
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
PclDatabaseMergeInterface::error() const
{
  return data->error;
}

/** Get maximum length of error value.
 * @return length of error value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseMergeInterface::maxlenof_error() const
{
  return 256;
}

/** Set error value.
 * 
      Error description if reconstruction fails.
    
 * @param new_error new error value
 */
void
PclDatabaseMergeInterface::set_error(const char * new_error)
{
  strncpy(data->error, new_error, sizeof(data->error)-1);
  data->error[sizeof(data->error)-1] = 0;
  data_changed = true;
}

/* =========== message create =========== */
Message *
PclDatabaseMergeInterface::create_message(const char *type) const
{
  if ( strncmp("MergeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new MergeMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
PclDatabaseMergeInterface::copy_values(const Interface *other)
{
  const PclDatabaseMergeInterface *oi = dynamic_cast<const PclDatabaseMergeInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(PclDatabaseMergeInterface_data_t));
}

const char *
PclDatabaseMergeInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class PclDatabaseMergeInterface::MergeMessage <interfaces/PclDatabaseMergeInterface.h>
 * MergeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_timestamps initial value for timestamps
 * @param ini_database initial value for database
 * @param ini_collection initial value for collection
 */
PclDatabaseMergeInterface::MergeMessage::MergeMessage(const int64_t * ini_timestamps, const char * ini_database, const char * ini_collection) : Message("MergeMessage")
{
  data_size = sizeof(MergeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MergeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  memcpy(data->timestamps, ini_timestamps, sizeof(int64_t) * 12);
  strncpy(data->database, ini_database, 64-1);
  data->database[64-1] = 0;
  strncpy(data->collection, ini_collection, 128-1);
  data->collection[128-1] = 0;
  add_fieldinfo(IFT_INT64, "timestamps", 12, &data->timestamps);
  add_fieldinfo(IFT_STRING, "database", 64, data->database);
  add_fieldinfo(IFT_STRING, "collection", 128, data->collection);
}
/** Constructor */
PclDatabaseMergeInterface::MergeMessage::MergeMessage() : Message("MergeMessage")
{
  data_size = sizeof(MergeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MergeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_INT64, "timestamps", 12, &data->timestamps);
  add_fieldinfo(IFT_STRING, "database", 64, data->database);
  add_fieldinfo(IFT_STRING, "collection", 128, data->collection);
}

/** Destructor */
PclDatabaseMergeInterface::MergeMessage::~MergeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PclDatabaseMergeInterface::MergeMessage::MergeMessage(const MergeMessage *m) : Message("MergeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (MergeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get timestamps value.
 * 
      Timestamps for which to retrieve the most recent point clouds
      and merge them.
    
 * @return timestamps value
 */
int64_t *
PclDatabaseMergeInterface::MergeMessage::timestamps() const
{
  return data->timestamps;
}

/** Get timestamps value at given index.
 * 
      Timestamps for which to retrieve the most recent point clouds
      and merge them.
    
 * @param index index of value
 * @return timestamps value
 * @exception Exception thrown if index is out of bounds
 */
int64_t
PclDatabaseMergeInterface::MergeMessage::timestamps(unsigned int index) const
{
  if (index > 12) {
    throw Exception("Index value %u out of bounds (0..12)", index);
  }
  return data->timestamps[index];
}

/** Get maximum length of timestamps value.
 * @return length of timestamps value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseMergeInterface::MergeMessage::maxlenof_timestamps() const
{
  return 12;
}

/** Set timestamps value.
 * 
      Timestamps for which to retrieve the most recent point clouds
      and merge them.
    
 * @param new_timestamps new timestamps value
 */
void
PclDatabaseMergeInterface::MergeMessage::set_timestamps(const int64_t * new_timestamps)
{
  memcpy(data->timestamps, new_timestamps, sizeof(int64_t) * 12);
}

/** Set timestamps value at given index.
 * 
      Timestamps for which to retrieve the most recent point clouds
      and merge them.
    
 * @param new_timestamps new timestamps value
 * @param index index for of the value
 */
void
PclDatabaseMergeInterface::MergeMessage::set_timestamps(unsigned int index, const int64_t new_timestamps)
{
  if (index > 12) {
    throw Exception("Index value %u out of bounds (0..12)", index);
  }
  data->timestamps[index] = new_timestamps;
}
/** Get database value.
 * 
      Database name from which to read the point clouds. If empty will
      use plugin-configured default.
    
 * @return database value
 */
char *
PclDatabaseMergeInterface::MergeMessage::database() const
{
  return data->database;
}

/** Get maximum length of database value.
 * @return length of database value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseMergeInterface::MergeMessage::maxlenof_database() const
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
PclDatabaseMergeInterface::MergeMessage::set_database(const char * new_database)
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
PclDatabaseMergeInterface::MergeMessage::collection() const
{
  return data->collection;
}

/** Get maximum length of collection value.
 * @return length of collection value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PclDatabaseMergeInterface::MergeMessage::maxlenof_collection() const
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
PclDatabaseMergeInterface::MergeMessage::set_collection(const char * new_collection)
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
PclDatabaseMergeInterface::MergeMessage::clone() const
{
  return new PclDatabaseMergeInterface::MergeMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
PclDatabaseMergeInterface::message_valid(const Message *message) const
{
  const MergeMessage *m0 = dynamic_cast<const MergeMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(PclDatabaseMergeInterface)
/// @endcond


} // end namespace fawkes
